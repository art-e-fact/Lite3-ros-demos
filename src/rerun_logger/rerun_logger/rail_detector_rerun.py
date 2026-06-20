"""Rerun logging for rail_detector MarkerArray messages."""

from __future__ import annotations

import re

import numpy as np
import rerun as rr
from rclpy.time import Time
from visualization_msgs.msg import Marker, MarkerArray


def log_rail_detector_markers(msg: MarkerArray, detector_frame: str | None) -> str | None:
    """
    Convert a rail_detector_node MarkerArray into Rerun entities.

    Each RViz marker namespace maps 1:1 to a ``detector/*`` Rerun path.
    Numeric metrics are extracted from the summary text marker so they can be
    plotted as Rerun scalars.

    Args:
        msg: The incoming MarkerArray from ``/rail_detector/markers``.
        detector_frame: The coordinate frame name last pinned to the detector
            entities, or ``None`` if not yet initialised.

    Returns:
        The (possibly updated) ``detector_frame`` value.
    """
    # Group markers by namespace; skip the leading DELETEALL sentinel.
    ns_markers: dict[str, list[Marker]] = {}
    frame_id: str | None = None
    stamp = None

    for marker in msg.markers:
        if marker.action == Marker.DELETEALL:
            continue
        ns_markers.setdefault(marker.ns, []).append(marker)
        if stamp is None:
            frame_id = marker.header.frame_id
            stamp = marker.header.stamp

    if stamp is None:
        # Message contained only a DELETEALL — clear all detector paths.
        for path in [
            'detector/slice_profiles', 'detector/slice_baselines', 'detector/rail_hits',
            'detector/center_samples', 'detector/follow_target_candidates',
            'detector/follow_target', 'detector/centerline',
            'detector/summary',
        ]:
            rr.log(path, [])
        return detector_frame

    time = Time.from_msg(stamp)
    rr.set_time("ros_time", timestamp=np.datetime64(time.nanoseconds, "ns"))

    # Pin every leaf entity to the correct named frame once, statically, so
    # Rerun can resolve the transform path even after dynamic clear calls.
    if frame_id and frame_id != detector_frame:
        detector_frame = frame_id
        for path in [
            'detector/slice_profiles', 'detector/slice_baselines', 'detector/rail_hits',
            'detector/center_samples', 'detector/follow_target_candidates',
            'detector/follow_target', 'detector/centerline',
            'detector/summary',
        ]:
            rr.log(path, rr.CoordinateFrame(frame=frame_id), static=True)

    # Slice profiles (one LINE_STRIP marker per slice)
    strips = [
        [[p.x, p.y, p.z] for p in m.points]
        for m in ns_markers.get('slice_profiles', [])
        if m.points
    ]
    if strips:
        rr.log('detector/slice_profiles', rr.LineStrips3D(strips, radii=0.0075, colors=[[51, 178, 255, 191]]))
    else:
        rr.log('detector/slice_profiles', [])

    baseline_strips = [
        [[p.x, p.y, p.z] for p in m.points]
        for m in ns_markers.get('slice_baselines', [])
        if m.points
    ]
    if baseline_strips:
        rr.log(
            'detector/slice_baselines',
            rr.LineStrips3D(baseline_strips, radii=0.0075, colors=[[255, 255, 166, 191]]),
        )
    else:
        rr.log('detector/slice_baselines', [])

    # Rail hits (single SPHERE_LIST marker)
    hits_markers = ns_markers.get('rail_hits', [])
    if hits_markers:
        pts = np.array([[p.x, p.y, p.z] for p in hits_markers[0].points])
        rr.log('detector/rail_hits', rr.Points3D(positions=pts, radii=0.04, colors=[[128, 51, 242]]))
    else:
        rr.log('detector/rail_hits', [])

    # Center samples / midpoints (single SPHERE_LIST marker)
    cs_markers = ns_markers.get('center_samples', [])
    if cs_markers:
        pts = np.array([[p.x, p.y, p.z] for p in cs_markers[0].points])
        rr.log('detector/center_samples', rr.Points3D(positions=pts, radii=0.05, colors=[[255, 230, 25]]))
    else:
        rr.log('detector/center_samples', [])

    # Follow-target candidates (single SPHERE_LIST marker)
    ftc_markers = ns_markers.get('follow_target_candidates', [])
    if ftc_markers:
        pts = np.array([[p.x, p.y, p.z] for p in ftc_markers[0].points])
        rr.log('detector/follow_target_candidates', rr.Points3D(positions=pts, radii=0.04, colors=[[255, 140, 25]]))
    else:
        rr.log('detector/follow_target_candidates', [])

    # Follow target (single CYLINDER marker)
    ft_markers = ns_markers.get('follow_target', [])
    if ft_markers:
        m = ft_markers[0]
        rr.log(
            'detector/follow_target',
            rr.Cylinders3D(
                lengths=[m.scale.z],
                radii=[m.scale.x / 2.0],
                centers=[[m.pose.position.x, m.pose.position.y, m.pose.position.z]],
                colors=[[
                    round(m.color.r * 255), round(m.color.g * 255),
                    round(m.color.b * 255), round(m.color.a * 255),
                ]],
            ),
        )
    else:
        rr.log('detector/follow_target', [])

    # Centerline (single LINE_STRIP marker)
    cl_markers = ns_markers.get('centerline', [])
    if cl_markers:
        pts = [[p.x, p.y, p.z] for p in cl_markers[0].points]
        rr.log('detector/centerline', rr.LineStrips3D([pts], radii=0.025, colors=[[25, 255, 51]]))
    else:
        rr.log('detector/centerline', [])

    # Summary text + scalar metrics (TEXT_VIEW_FACING marker)
    summary_markers = ns_markers.get('summary', [])
    if summary_markers:
        text = summary_markers[0].text
        summary_md = '# Rail Detector Summary\n\n'

        m = re.search(r'offset=([+-]?\d+\.\d+)', text)
        if m:
            offset = float(m.group(1))
            summary_md += f'* **Center Offset**: `{offset:+.2f} m`\n'
            rr.log('detector/metrics/center_offset', rr.Scalars(offset))

        m = re.search(r'heading=([+-]?\d+\.\d+)', text)
        if m:
            heading = float(m.group(1))
            summary_md += f'* **Heading (Yaw)**: `{heading:.1f}°`\n'
            rr.log('detector/metrics/heading_deg', rr.Scalars(heading))

        if 'rail parse incomplete' in text:
            summary_md += '* **Status**: `rail parse incomplete`\n'

        m = re.search(r'follow_target=(\d+\.\d+) m \(h=(\d+\.\d+) m\)', text)
        if m:
            dist, height = float(m.group(1)), float(m.group(2))
            summary_md += '* **Follow Target**:\n'
            summary_md += f'  * **Distance**: `{dist:.2f} m`\n'
            summary_md += f'  * **Height**: `{height:.2f} m`\n'
            rr.log('detector/metrics/target_distance', rr.Scalars(dist))
            rr.log('detector/metrics/target_height', rr.Scalars(height))
        elif 'follow_target=none' in text:
            summary_md += '* **Follow Target**: `none`\n'

        rr.log('detector/summary', rr.TextDocument(summary_md, media_type=rr.MediaType.MARKDOWN))
    else:
        rr.log('detector/summary', [])

    return detector_frame
