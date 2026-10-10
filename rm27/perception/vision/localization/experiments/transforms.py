"""Explicit derived-image stress experiments; no source mutation or sensor model."""
from copy import deepcopy
from pathlib import Path

import cv2
import numpy as np

from .common import checked_file, number, require, write_json, object_hash, file_hash


def prepare_images(frames, calibration, destination, config):
    require(set(config) <= {'grayscale', 'scale', 'blur_sigma', 'brightness_gain', 'brightness_offset', 'mask'},
            'TRANSFORM_UNSUPPORTED', 'unknown transform key')
    require(type(config.get('grayscale', False)) is bool, 'TRANSFORM_INVALID', 'grayscale')
    scale = number(config.get('scale', 1), 'scale', True)
    require(scale <= 1, 'TRANSFORM_INVALID', 'downscale only')
    sigma = number(config.get('blur_sigma', 0), 'blur_sigma')
    require(0 <= sigma <= 20, 'TRANSFORM_INVALID', 'blur_sigma must be 0..20 pixels')
    gain = number(config.get('brightness_gain', 1), 'brightness_gain', True)
    offset = number(config.get('brightness_offset', 0), 'brightness_offset')
    width, height = frames[0].width, frames[0].height
    new_width, new_height = round(width*scale), round(height*scale)
    require(min(new_width, new_height) >= 2, 'TRANSFORM_INVALID', 'image too small')
    mask = None
    if config.get('mask') is not None:
        reference = config['mask']
        require(reference.get('status') == 'VALIDATED' and reference.get('sha256'),
                'MASK_NOT_VALIDATED', 'supply an identified reviewed binary mask, otherwise NONE')
        path, digest = checked_file(reference['file'], reference['sha256'])
        mask = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        require(mask is not None and mask.shape == (height, width) and set(np.unique(mask)) <= {0, 255},
                'MASK_INVALID', 'source-resolution binary 0/255 mask required')
    changed = config.get('grayscale', False) or scale != 1 or sigma != 0 or gain != 1 or offset != 0 or mask is not None
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=False)
    output, records = [], []
    for i, frame in enumerate(frames):
        checked_file(frame.image_path, frame.image_sha256)
        suffix = '.png' if changed else frame.image_path.suffix
        target = destination/f'{i:08d}{suffix}'
        if changed:
            image = cv2.imread(str(frame.image_path), cv2.IMREAD_COLOR)
            require(image is not None, 'IMAGE_DECODE_FAILED', str(frame.image_path))
            if config.get('grayscale', False):
                image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            if scale != 1:
                image = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
            if sigma:
                image = cv2.GaussianBlur(image, (0, 0), sigma)
            if gain != 1 or offset != 0:
                image = np.clip(image.astype(float)*gain+offset, 0, 255).astype(np.uint8)
            if mask is not None:
                effective_mask = cv2.resize(mask, (new_width, new_height), interpolation=cv2.INTER_NEAREST)
                image[effective_mask == 0] = 0
            require(cv2.imwrite(str(target), image), 'IMAGE_WRITE_FAILED', str(target))
        else:
            target.symlink_to(frame.image_path)
        output.append(target)
        records.append(dict(source_frame_id=frame.source_frame_id, source_image_sha256=frame.image_sha256,
                            image=str(target), image_sha256=file_hash(target), encoded_pts=frame.encoded_pts))
    derived_calibration = deepcopy(calibration)
    sx, sy = new_width/width, new_height/height
    derived_calibration.update(image_width=new_width, image_height=new_height,
                               fx=calibration['fx']*sx, fy=calibration['fy']*sy,
                               cx=(calibration['cx']+.5)*sx-.5, cy=(calibration['cy']+.5)*sy-.5)
    provenance = dict(config=config, config_sha256=object_hash(config), frames=records,
                      output_geometry=[new_width, new_height],
                      resize_mapping='OpenCV linear pixel centers: (coordinate + 0.5)*scale - 0.5',
                      mask_semantic='image-domain black-out stress; not backend feature exclusion' if mask is not None else 'NONE',
                      sensor_physics_validated=False)
    write_json(destination.parent/'derived_inputs.json', provenance)
    return output, derived_calibration, provenance
