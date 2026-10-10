"""One pinned public RGB sequence for VSL-3P; no RM27 hardware qualification.

The provider calibration is imported, not fitted or independently validated.
The archive pin below is a locally measured download SHA-256, not a publisher
signature. Original RGB timestamps and image bytes are retained.
"""
import argparse
import csv
from decimal import Decimal
from pathlib import Path
import tarfile
import hashlib
import math

from .common import checked_file, file_hash, read_json, write_json, require, resolve

DATASET_ID = 'TUM_RGBD_freiburg1_xyz'
ARCHIVE_SHA256 = 'a0236d97b8c30cd93b653656d2b6c293ff7c982a4130ef2a1a8beecdb124ef98'
SOURCE_URL = 'https://cvg.cit.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.tgz'
CALIBRATION_URL = 'https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats'
CALIBRATION_ID = 'TUM_freiburg1_RGB_published'
PARAMETERS = dict(model='pinhole', distortion_model='opencv_radtan5', image_width=640,
                  image_height=480, fx=517.3, fy=516.5, cx=318.6, cy=255.3,
                  distortion_coefficients=[0.2624, -0.9531, -0.0054, 0.0026, 1.1633])


def published_calibration(dataset):
    """Separate provider-calibration scope. Never accepts a local camera dataset."""
    m = dataset.manifest
    require(m.get('dataset_id') == DATASET_ID and m.get('purpose') == 'VSL-3P_PUBLIC_DATASET'
            and m.get('status') == 'PUBLIC_REFERENCE', 'PUBLIC_DATASET_INVALID', 'pinned TUM qualification only')
    source = m['source']
    require(source.get('url') == SOURCE_URL and source.get('source_sha256') == ARCHIVE_SHA256
            and dataset.provenance['source_sha256'] == ARCHIVE_SHA256,
            'PUBLIC_DATASET_INVALID', 'public archive identity')
    ref = m.get('calibration', {})
    require(ref.get('status') == 'PUBLISHED' and ref.get('calibration_id') == CALIBRATION_ID,
            'CALIBRATION_NOT_VALIDATED', 'public provider reference required')
    path, digest = checked_file(resolve(dataset.path.parent, ref['file']), ref['sha256'])
    c = read_json(path)
    require(c.get('scope') == 'PUBLIC_PROVIDER_CALIBRATION' and c.get('status') == 'PUBLISHED'
            and c.get('calibration_id') == CALIBRATION_ID and c.get('dataset_id') == DATASET_ID
            and all(c.get(k) == v for k, v in PARAMETERS.items()),
            'CALIBRATION_CONFIGURATION_MISMATCH', 'published Freiburg1 RGB parameters')
    g = m['observed_geometry']
    require((g['width'], g['height']) == (640, 480) and g.get('pixel_geometry') == {'crop': 'none', 'resize': 'none'}
            and all(f.calibration_id == CALIBRATION_ID for f in dataset.frames),
            'CALIBRATION_GEOMETRY_MISMATCH', 'original Freiburg1 RGB geometry only')
    evidence = c['provider_evidence']
    require(evidence.get('url') == CALIBRATION_URL, 'CALIBRATION_PROVENANCE_MISSING', 'provider URL')
    doc, doc_hash = checked_file(resolve(path.parent, evidence['file']), evidence['sha256'])
    require(all(str(v) in doc.read_text() for v in [517.3, 516.5, 318.6, 255.3, 0.2624, -0.9531, 1.1633]),
            'CALIBRATION_PROVENANCE_MISSING', 'retained provider parameter table')
    return c, dict(path=str(path), sha256=digest, calibration_id=CALIBRATION_ID,
                   scope='PUBLIC_PROVIDER_CALIBRATION', provider_document=str(doc),
                   provider_document_sha256=doc_hash, physical_rm27_calibration=False)


def import_sequence(archive, sequence, provider_document, output):
    archive, digest = checked_file(archive, ARCHIVE_SHA256)
    sequence, output = Path(sequence).resolve(), Path(output).resolve()
    doc, doc_hash = checked_file(provider_document)
    output.mkdir(parents=True, exist_ok=False)
    rows = [line.split() for line in (sequence/'rgb.txt').read_text().splitlines()
            if line.strip() and not line.startswith('#')]
    # Verify extracted RGB files and reference against the hashed original archive.
    hashes = {}
    wanted = {'rgb.txt', 'groundtruth.txt', *[r[1] for r in rows]}
    with tarfile.open(archive, 'r|gz') as tar:
        for entry in tar:
            name = entry.name.removeprefix('rgbd_dataset_freiburg1_xyz/')
            if name not in wanted:
                continue
            member = tar.extractfile(entry)
            expected = hashlib.file_digest(member, 'sha256').hexdigest()
            checked_file(sequence/name, expected)
            hashes[name] = expected
    require(set(hashes) == wanted, 'PUBLIC_DATASET_INVALID', 'archive members missing')
    calibration = dict(schema_version=1, status='PUBLISHED', scope='PUBLIC_PROVIDER_CALIBRATION',
                       dataset_id=DATASET_ID, calibration_id=CALIBRATION_ID, **PARAMETERS,
                       camera_module='TUM Freiburg1 RGB camera', lens_id='UNKNOWN', focus_setting='UNKNOWN',
                       camera_mode_id='published original RGB 640x480',
                       pixel_geometry={'crop':'none','resize':'none'},
                       provider_evidence=dict(url=CALIBRATION_URL,file=str(doc),sha256=doc_hash),
                       independent_rm27_calibration_validation=False)
    write_json(output/'calibration.json', calibration)
    names=['sample_index','source_frame_index','encoded_pts_s','image','image_sha256','width','height',
           'calibration_id','timestamp_unit','timestamp_clock_domain','timestamp_semantic','timestamp_evidence_status']
    with (output/'frames.csv').open('w') as f:
        writer=csv.DictWriter(f, fieldnames=names); writer.writeheader()
        for i,(timestamp,name) in enumerate(rows):
            writer.writerow(dict(zip(names,[i,i,timestamp,name,hashes[name],640,480,CALIBRATION_ID,
                                            's','TUM_dataset_clock','DATASET_TIMESTAMP','PROVIDER_DOCUMENTED'])))
    write_json(output/'intervals.json', [])
    write_json(output/'subsets.json', {'all_decoded_sampled':dict(frame_count=len(rows),source_frame_indices=list(range(len(rows))))})
    m=dict(schema_version=1,dataset_id=DATASET_ID,dataset_kind='TUM_RGBD_PUBLIC',purpose='VSL-3P_PUBLIC_DATASET',
           status='PUBLIC_REFERENCE',frame_count=len(rows),frames_file='frames.csv',frames_sha256=file_hash(output/'frames.csv'),
           intervals_file='intervals.json',subsets_file='subsets.json',
           source=dict(artifact_directory=str(sequence),source_archive=str(archive),source_sha256=digest,url=SOURCE_URL,
                       upstream_metadata_sha256={k:hashes[k] for k in ['rgb.txt','groundtruth.txt']}),
           observed_geometry=dict(width=640,height=480,nominal_fps=30,pixel_geometry={'crop':'none','resize':'none'}),
           calibration=dict(status='PUBLISHED',calibration_id=CALIBRATION_ID,file='calibration.json',sha256=file_hash(output/'calibration.json')))
    write_json(output/'dataset_manifest.json',m)
    # Nearest reference within 20 ms; one-to-one, no timestamp shift or fitted offset.
    truth=[line.split() for line in (sequence/'groundtruth.txt').read_text().splitlines()
           if line.strip() and not line.startswith('#')]
    candidates=[]
    for i,(timestamp,_) in enumerate(rows):
        for j,gt in enumerate(truth):
            delta=abs(Decimal(timestamp)-Decimal(gt[0]))
            if delta <= Decimal('.02'): candidates.append((delta,i,j))
    used_rgb,used_gt,records=set(),set(),[]
    for delta,i,j in sorted(candidates):
        if i in used_rgb or j in used_gt: continue
        used_rgb.add(i);used_gt.add(j)
        gt=truth[j];q=[float(gt[7]),*map(float,gt[4:7])];norm=math.hypot(*q)
        require(abs(norm-1)<.001,'REFERENCE_INVALID','rounded TUM reference quaternion')
        records.append(dict(source_frame_id=i,encoded_pts=rows[i][0],trajectory_variant='reference',
                            reference_timestamp=gt[0],association_delta_s=float(delta),reference_quaternion_norm=norm,
                            localization_fields=dict(parent_frame='TUM_mocap_world',child_frame='camera',
                            translation=list(map(float,gt[1:4])),orientation_wxyz=[v/norm for v in q])))
    write_json(output/'reference.json',dict(dataset_id=DATASET_ID,dataset_manifest_sha256=file_hash(output/'dataset_manifest.json'),
               reference_status='VERIFIED',timestamp_association='VERIFIED',translation_unit='m',
               source_file=str(sequence/'groundtruth.txt'),source_sha256=hashes['groundtruth.txt'],
               association_policy='global nearest one-to-one within 20ms, zero offset; no interpolation',
               quaternion_policy='normalize rounded provider reference quaternions; norms retained',
               records=sorted(records,key=lambda r:r['source_frame_id'])))
    return output/'dataset_manifest.json'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['archive','sequence','provider-document','output']:
        parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    print(import_sequence(args.archive,args.sequence,args.provider_document,args.output))


if __name__=='__main__': main()
