"""Rerun visualization helper functions for the rail detector."""

import math

import numpy as np

import rerun as rr


def log_to_rerun(detection, forward_span):
    """Log rail detector states to Rerun."""
    # 1. Slice Profiles
    slice_strips = []
    for slice_result in detection['slices']:
        points = [
            [float(x), float(y), float(z) + 0.01]
            for (x, y), z in zip(slice_result['xy'], slice_result['z'])
            if math.isfinite(float(z))
        ]
        if points:
            slice_strips.append(points)

    if slice_strips:
        rr.log(
            'detector/slice_profiles',
            rr.LineStrips3D(
                slice_strips,
                radii=0.0075,  # half of scale.x=0.015
                colors=[[51, 178, 255, 191]],  # r=0.2, g=0.7, b=1.0, a=0.75
            ),
        )
    else:
        # Clear if empty
        rr.log('detector/slice_profiles', [])

    # 2. Rail Hits
    if len(detection['hits']) != 0:
        hits = np.asarray(detection['hits'])
        # Add 0.02 to Z to match RViz height adjustment
        hits_adjusted = hits.copy()
        hits_adjusted[:, 2] += 0.02
        rr.log(
            'detector/rail_hits',
            rr.Points3D(
                positions=hits_adjusted,
                radii=0.04,  # half of scale=0.08
                colors=[[128, 51, 242]],  # r=0.5, g=0.2, b=0.95
            ),
        )
    else:
        rr.log('detector/rail_hits', [])

    # 3. Center Samples (Midpoints)
    if len(detection['midpoints']) != 0:
        midpoints = np.asarray(detection['midpoints'])
        midpoints_adjusted = midpoints.copy()
        midpoints_adjusted[:, 2] += 0.02
        rr.log(
            'detector/center_samples',
            rr.Points3D(
                positions=midpoints_adjusted,
                radii=0.05,  # half of scale=0.10
                colors=[[255, 230, 25]],  # r=1.0, g=0.9, b=0.1
            ),
        )
    else:
        rr.log('detector/center_samples', [])

    # 4. Follow Target Candidates
    if len(detection['follow_target_points']) != 0:
        candidates = np.asarray(detection['follow_target_points'])
        candidates_adjusted = candidates.copy()
        candidates_adjusted[:, 2] += 0.02
        rr.log(
            'detector/follow_target_candidates',
            rr.Points3D(
                positions=candidates_adjusted,
                radii=0.04,  # half of scale=0.08
                colors=[[255, 140, 25]],  # r=1.0, g=0.55, b=0.1
            ),
        )
    else:
        rr.log('detector/follow_target_candidates', [])

    # 5. Follow Target
    if detection['follow_target'] is not None:
        target_pt = detection['follow_target']['point']
        rr.log(
            'detector/follow_target',
            rr.Cylinders3D(
                lengths=[1.8],
                radii=[0.17],  # half of diameter=0.34
                centers=[[float(target_pt[0]), float(target_pt[1]), 0.9]],
                colors=[[255, 128, 0, 128]],  # r=1.0, g=0.5, b=0.0, a=0.5
            ),
        )
    else:
        rr.log('detector/follow_target', [])

    # 6. Centerline
    z_level = _marker_height(
        detection['midpoints'],
        detection['hits'],
        detection['follow_target_points'],
    )
    if detection['line'] is not None:
        line = detection['line']
        start = line['center'] - 0.8 * forward_span * line['tangent']
        end = line['center'] + 0.8 * forward_span * line['tangent']
        rr.log(
            'detector/centerline',
            rr.LineStrips3D(
                [[
                    [float(start[0]), float(start[1]), float(z_level) + 0.04],
                    [float(end[0]), float(end[1]), float(z_level) + 0.04],
                ]],
                radii=0.025,  # half of width=0.05
                colors=[[25, 255, 51]],  # r=0.1, g=1.0, b=0.2
            ),
        )
    else:
        rr.log('detector/centerline', [])

    # 7. Summary Text Document & Metrics
    summary_md = '# Rail Detector Summary\n\n'
    if detection['line'] is not None:
        line = detection['line']
        offset = float(line['signed_offset'])
        heading = math.degrees(float(line['yaw']))
        summary_md += f'* **Center Offset**: `{offset:+.2f} m`\n'
        summary_md += f'* **Heading (Yaw)**: `{heading:.1f}°`\n'
        rr.log('detector/metrics/center_offset', rr.Scalars(offset))
        rr.log('detector/metrics/heading_deg', rr.Scalars(heading))
    else:
        summary_md += '* **Status**: `rail parse incomplete`\n'

    if detection['follow_target'] is not None:
        target = detection['follow_target']
        distance = float(target['distance'])
        height = float(target['height'])
        summary_md += '* **Follow Target**:\n'
        summary_md += f'  * **Distance**: `{distance:.2f} m`\n'
        summary_md += f'  * **Height**: `{height:.2f} m`\n'
        rr.log('detector/metrics/target_distance', rr.Scalars(distance))
        rr.log('detector/metrics/target_height', rr.Scalars(height))
    else:
        summary_md += '* **Follow Target**: `none`\n'

    rr.log(
        'detector/summary',
        rr.TextDocument(summary_md, media_type=rr.MediaType.MARKDOWN),
    )


def _marker_height(midpoints, hits, follow_target_points):
    if len(midpoints) != 0:
        return float(np.nanmedian(midpoints[:, 2]))
    if len(hits) != 0:
        return float(np.nanmedian(hits[:, 2]))
    if len(follow_target_points) != 0:
        return float(np.nanmedian(follow_target_points[:, 2]))
    return 0.0
