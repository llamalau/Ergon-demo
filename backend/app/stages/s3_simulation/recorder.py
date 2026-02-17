"""Record telemetry data and video from simulation runs."""

import io
import json
import numpy as np


def save_telemetry(telemetry: dict, job_id: str, trial_index: int, task_type: str) -> tuple[bytes, dict]:
    """Serialize telemetry data to numpy-compatible format.

    Returns (serialized bytes, summary dict).
    """
    # Convert lists to numpy arrays for efficient storage
    arrays = {}
    for key, value in telemetry.items():
        if isinstance(value, list) and len(value) > 0:
            try:
                arrays[key] = np.array(value)
            except (ValueError, TypeError):
                pass

    # Save as npz
    buf = io.BytesIO()
    np.savez_compressed(buf, **arrays)
    buf.seek(0)

    # Compute summary statistics
    summary = {
        "num_timesteps": len(telemetry.get("timesteps", [])),
        "duration": telemetry["timesteps"][-1] if telemetry.get("timesteps") else 0,
    }

    # Contact force stats
    if "contact_forces" in arrays:
        forces = arrays["contact_forces"]
        force_magnitudes = np.linalg.norm(forces[:, :3], axis=1)
        summary["max_contact_force"] = float(np.max(force_magnitudes))
        summary["mean_contact_force"] = float(np.mean(force_magnitudes))

    # Object displacement
    if "object_positions" in arrays:
        positions = arrays["object_positions"]
        displacement = np.linalg.norm(positions[-1] - positions[0])
        summary["object_displacement"] = float(displacement)
        summary["max_height"] = float(np.max(positions[:, 2]))

    return buf.getvalue(), summary


def encode_video(frames: list[np.ndarray], fps: int = 30) -> bytes | None:
    """Encode frames to MP4 video using imageio/ffmpeg.

    Returns MP4 bytes or None if no frames.
    """
    if not frames:
        return None

    try:
        import os
        import tempfile

        import imageio

        # imageio-ffmpeg requires writing to a file path (not BytesIO)
        with tempfile.NamedTemporaryFile(suffix=".mp4", delete=False) as tmp:
            tmp_path = tmp.name

        try:
            writer = imageio.get_writer(
                tmp_path, fps=fps, codec="libx264",
                output_params=["-pix_fmt", "yuv420p"],
            )
            for frame in frames:
                if frame.dtype != np.uint8:
                    frame = (frame * 255).astype(np.uint8)
                writer.append_data(frame)
            writer.close()

            with open(tmp_path, "rb") as f:
                return f.read()
        finally:
            if os.path.exists(tmp_path):
                os.unlink(tmp_path)
    except Exception as e:
        import logging
        logging.getLogger(__name__).warning("Video encoding failed: %s", e)
        return None
