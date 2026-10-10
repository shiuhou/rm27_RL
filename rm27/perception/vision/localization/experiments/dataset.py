"""Read canonical dependent views without copying images or changing PTS."""
from dataclasses import dataclass, asdict
import csv
from decimal import Decimal, InvalidOperation
from pathlib import Path

import cv2

from .common import checked_file, require, read_json, resolve, object_hash, number


@dataclass(frozen=True)
class Frame:
    sample_index: int
    source_frame_id: int
    image_path: Path
    image_sha256: str
    encoded_pts: str
    width: int
    height: int
    calibration_id: str
    timestamp_evidence: dict
    intervals: tuple

    def record(self):
        return {**asdict(self), 'image_path': str(self.image_path)}


class CanonicalDataset:
    """Validates the complete view before selection; no calibration qualification."""
    def __init__(self, manifest_path):
        self.path, self.manifest_hash = checked_file(manifest_path)
        self.manifest = m = read_json(self.path)
        require(type(m.get('schema_version')) is int and m['schema_version'] == 1,
                'DATASET_SCHEMA_UNSUPPORTED', 'expected canonical view version 1')
        require(isinstance(m.get('dataset_id'), str) and m['dataset_id'], 'DATASET_INVALID', 'dataset_id')
        base = self.path.parent
        source = m['source']
        artifact = resolve(base, source['artifact_directory'])
        source_name = 'source_archive' if m.get('dataset_kind') == 'TUM_RGBD_PUBLIC' else 'source_video'
        video, video_hash = checked_file(resolve(base, source[source_name]), source.get('source_sha256'))
        provenance = {'manifest_sha256': self.manifest_hash, 'source_video': str(video),
                      'source_sha256': video_hash, 'artifact_directory': str(artifact),
                      'upstream_identity': source, 'dependencies': {}}
        for name, expected in source.get('upstream_metadata_sha256', {}).items():
            p, h = checked_file(resolve(artifact, name), expected)
            provenance['dependencies'][str(p)] = h
        frames_file, h = checked_file(resolve(base, m['frames_file']), m.get('frames_sha256'))
        provenance['dependencies'][str(frames_file)] = h
        intervals_file, h = checked_file(resolve(base, m['intervals_file']), m.get('intervals_sha256'))
        provenance['dependencies'][str(intervals_file)] = h
        self.intervals = read_json(intervals_file)
        require(isinstance(self.intervals, list), 'DATASET_INVALID', 'intervals must be a list')
        for interval in self.intervals:
            a, b = number(interval['start_time_s'], 'interval start'), number(interval['end_time_s'], 'interval end')
            require(a <= b and isinstance(interval['label'], str), 'DATASET_INVALID', 'invalid interval')
        subset_file, h = checked_file(resolve(base, m.get('subsets_file', 'subsets.json')), m.get('subsets_sha256'))
        provenance['dependencies'][str(subset_file)] = h
        self.subsets = read_json(subset_file)
        hashes = {}
        hash_name = source.get('image_hashes_file', m.get('image_hashes_file'))
        if hash_name:
            p, h = checked_file(resolve(base, hash_name), source.get('image_hashes_sha256'))
            provenance['dependencies'][str(p)] = h
            hashes = read_json(p)
        with frames_file.open(newline='') as stream:
            rows = list(csv.DictReader(stream))
        require(rows and len(rows) == m['frame_count'], 'DATASET_INVALID', 'frame count mismatch/empty')
        self.frames = []
        previous_id, previous_pts = -1, None
        geom = m['observed_geometry']
        require(all(type(geom.get(k)) is int and geom[k] > 0 for k in ('width', 'height')), 'GEOMETRY_MISMATCH', 'positive integer geometry required')
        number(geom.get('nominal_fps'), 'nominal_fps', True)
        for i, row in enumerate(rows):
            source_id = int(row['source_frame_index'])
            require(int(row['sample_index']) == i and source_id > previous_id,
                    'FRAME_ORDER_INVALID', 'sample or source frame order')
            try:
                pts = Decimal(row['encoded_pts_s'])
            except InvalidOperation:
                require(False, 'TIMESTAMP_INVALID', 'encoded PTS')
            require(pts.is_finite() and pts >= 0 and (previous_pts is None or pts > previous_pts),
                    'FRAME_ORDER_INVALID', 'one strictly increasing encoded timeline is required')
            timing = dict(unit=row['timestamp_unit'], clock_domain=row['timestamp_clock_domain'],
                          semantic=row['timestamp_semantic'], evidence_status=row.get('timestamp_evidence_status') or 'UNVERIFIED',
                          raw_value=row['encoded_pts_s'], representation='original decimal seconds; semantic stated separately')
            expected_semantic = 'DATASET_TIMESTAMP' if m.get('dataset_kind') == 'TUM_RGBD_PUBLIC' else 'ENCODED_STREAM_PTS'
            require(timing['unit'] in ('s', 'seconds') and timing['semantic'] == expected_semantic,
                    'TIMESTAMP_UNSUPPORTED', 'reader only supports encoded seconds, never exposure')
            if self.frames:
                previous_evidence = self.frames[-1].timestamp_evidence
                require(all(timing[k] == previous_evidence[k] for k in ('unit', 'clock_domain', 'semantic')),
                        'TIMESTAMP_UNSUPPORTED', 'one encoded timeline per dataset view')
            width, height = int(row['width']), int(row['height'])
            require((width, height) == (geom['width'], geom['height']), 'GEOMETRY_MISMATCH', 'frame metadata')
            p, h = checked_file(resolve(artifact, row['image']), hashes.get(row['image'], row.get('image_sha256') or None))
            if hash_name:
                require(row['image'] in hashes, 'HASH_MISSING', str(p))
            image = cv2.imread(str(p), cv2.IMREAD_UNCHANGED)
            require(image is not None and image.shape[:2] == (height, width), 'GEOMETRY_MISMATCH', str(p))
            labels = tuple(x['label'] for x in self.intervals if Decimal(str(x['start_time_s'])) <= pts <= Decimal(str(x['end_time_s'])))
            self.frames.append(Frame(i, source_id, p, h, row['encoded_pts_s'], width, height,
                                     row['calibration_id'], timing, labels))
            previous_id, previous_pts = source_id, pts
        known_ids = {f.source_frame_id for f in self.frames}
        for name, subset in self.subsets.items():
            ids = subset['source_frame_indices']
            require(all(type(i) is int for i in ids) and ids == sorted(set(ids)) and set(ids) <= known_ids
                    and len(ids) == subset['frame_count'], 'SUBSET_INVALID', name)
        provenance['frame_images_identity'] = object_hash([(f.source_frame_id, f.image_sha256) for f in self.frames])
        self.provenance = provenance

    def select(self, subset='all_decoded_sampled', input_hz=None):
        require(subset in self.subsets, 'SUBSET_NOT_FOUND', subset)
        ids = set(self.subsets[subset]['source_frame_indices'])
        if input_hz is not None:
            source_hz = number(self.manifest['observed_geometry']['nominal_fps'], 'source FPS', True)
            number(input_hz, 'input_hz', True)
            ratio = source_hz/input_hz
            require(ratio >= 1 and abs(ratio-round(ratio)) < 1e-9, 'RATE_UNSUPPORTED', 'integer source-frame decimation required')
            step = round(ratio)
            # Verify requested grid exists globally, before any sparse diagnostic subset.
            present = {f.source_frame_id for f in self.frames}
            expected = set(range(0, max(present)+1, step))
            require(expected <= present, 'RATE_UNAVAILABLE', 'this view lacks requested source frames; extract a denser view')
            ids &= expected
        selected = [f for f in self.frames if f.source_frame_id in ids]
        require(len(selected) >= 2, 'SELECTION_EMPTY', 'at least two frames required')
        return selected
