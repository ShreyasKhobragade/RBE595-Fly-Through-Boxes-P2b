import os
import subprocess
import sys

def make_video_ffmpeg(
    frames_dir="./renders/rgb",
    output_path="./renders/output_video.mp4",
    fps=30,
    pattern="frame_%05d.png"
):
    """
    Convert numbered PNG frames into a video using ffmpeg.

    Args:
        frames_dir (str): Path to folder containing frames.
        output_path (str): Output video file path.
        fps (int): Frames per second.
        pattern (str): Naming pattern, e.g. frame_%05d.png for frame_00001.png.
    """

    # Ensure directory exists
    if not os.path.isdir(frames_dir):
        print(f"❌ Error: Directory not found: {frames_dir}")
        sys.exit(1)

    # Check if ffmpeg is installed
    try:
        subprocess.run(["ffmpeg", "-version"], check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except FileNotFoundError:
        print("❌ FFmpeg is not installed or not found in PATH.")
        print("👉 Install it via: sudo apt install ffmpeg  (Linux) or brew install ffmpeg (macOS)")
        sys.exit(1)

    # Construct input and output paths
    input_pattern = os.path.join(frames_dir, pattern)
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    # FFmpeg command
    cmd = [
        "ffmpeg",
        "-y",                        # overwrite existing file
        "-framerate", str(fps),      # input frame rate
        "-i", input_pattern,         # input frame sequencet
        "-c:v", "libx264",           # video codec (H.264)
        "-pix_fmt", "yuv420p",       # pixel format for compatibility
        output_path
    ]

    print(f"🎥 Generating video from frames in: {frames_dir}")
    print(f"➡️  Output: {output_path}")
    print(f"⚙️  Command: {' '.join(cmd)}")

    # Run ffmpeg
    try:
        subprocess.run(cmd, check=True)
        print(f"✅ Video successfully saved at: {output_path}")
    except subprocess.CalledProcessError as e:
        print(f"❌ FFmpeg failed with error:\n{e}")
        sys.exit(1)


if __name__ == "__main__":
    # Example usage
    make_video_ffmpeg(
        frames_dir="./renders/rgb",       # Folder containing your PNGs
        output_path="./renders/simulation.mp4",  # Output video path
        fps=30,                           # Frame rate
        pattern="frame_%05d.png"          # Match frame_00001.png, etc.
    )
