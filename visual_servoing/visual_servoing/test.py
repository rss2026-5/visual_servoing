import numpy as np
import cv2
import matplotlib.pyplot as plt

METERS_PER_INCH = 0.0254

PTS_IMAGE_PLANE = [[403, 286],
                   [203, 280],
                   [378, 249],
                   [569, 279],
                   [366, 195]]

PTS_GROUND_PLANE = [[10,  0],
                    [10,  10],
                    [15,  0],
                    [10, -10],
                    [35,  0]]


def compute_homography_and_error():
    np_pts_ground = np.array(PTS_GROUND_PLANE, dtype=np.float32) * METERS_PER_INCH
    np_pts_image  = np.array(PTS_IMAGE_PLANE,  dtype=np.float32)
    h, _ = cv2.findHomography(np_pts_image[:, np.newaxis, :], np_pts_ground[:, np.newaxis, :])

    errors, pred_points = [], []
    for uv, xy in zip(PTS_IMAGE_PLANE, PTS_GROUND_PLANE):
        pt = np.array([[uv[0]], [uv[1]], [1.0]])
        proj = h @ pt
        proj /= proj[2]
        pred_x, pred_y = proj[0, 0], proj[1, 0]
        actual_x = xy[0] * METERS_PER_INCH
        actual_y = xy[1] * METERS_PER_INCH
        err = np.sqrt((pred_x - actual_x)**2 + (pred_y - actual_y)**2) * 100  # cm
        errors.append(err)
        pred_points.append((pred_x, pred_y, actual_x, actual_y))
        print(f"  pixel={uv}  predicted=({pred_x:.4f}, {pred_y:.4f})m  actual=({actual_x:.4f}, {actual_y:.4f})m  error={err:.2f}cm")

    print(f"\nMean error: {np.mean(errors):.2f} cm  |  Max error: {np.max(errors):.2f} cm")
    return errors, pred_points


def plot_errors(errors, pred_points):
    n = len(PTS_IMAGE_PLANE)
    x = np.arange(n)
    labels = [f"P{i}\n{PTS_IMAGE_PLANE[i]}" for i in range(n)]

    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle("Homography Reprojection Error", fontsize=14, fontweight="bold")

    # --- Bar chart ---
    ax = axes[0]
    bars = ax.bar(x, errors, color="steelblue", alpha=0.85)
    for bar, val in zip(bars, errors):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.005,
                f"{val:.2f} cm", ha="center", va="bottom", fontsize=9)
    ax.axhline(1.0, color="tomato", linestyle="--", linewidth=1, label="1 cm threshold")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=8)
    ax.set_xlabel("Calibration Point")
    ax.set_ylabel("Reprojection Error (cm)")
    ax.set_title("Per-point Error")
    ax.set_ylim(0, max(errors) * 1.4 + 0.1)
    ax.legend()

    # --- Ground plane scatter ---
    ax2 = axes[1]
    actual_xs = [p[2] for p in pred_points]
    actual_ys = [p[3] for p in pred_points]
    pred_xs   = [p[0] for p in pred_points]
    pred_ys   = [p[1] for p in pred_points]

    ax2.scatter(actual_xs, actual_ys, marker="*", s=220, color="black", zorder=5, label="Actual")
    ax2.scatter(pred_xs,   pred_ys,   marker="o", s=80,  color="steelblue", zorder=4, label="Predicted")

    for i, (ax_, ay, px, py) in enumerate(zip(actual_xs, actual_ys, pred_xs, pred_ys)):
        ax2.annotate("", xy=(px, py), xytext=(ax_, ay),
                     arrowprops=dict(arrowstyle="->", color="steelblue", lw=1.3))
        ax2.text(ax_ + 0.005, ay + 0.005, f"P{i}", fontsize=8)

    ax2.set_xlabel("X — forward (m)")
    ax2.set_ylabel("Y — lateral (m)")
    ax2.set_title("Ground Plane: Actual vs Predicted")
    ax2.legend(fontsize=9)
    ax2.set_aspect("equal")
    ax2.grid(True, linestyle="--", alpha=0.4)
    ax2.axhline(0, color="gray", lw=0.5)
    ax2.axvline(0, color="gray", lw=0.5)

    plt.tight_layout()
    plt.savefig("homography_error.png", dpi=150)
    print("Saved to homography_error.png")
    plt.show()


errors, pred_points = compute_homography_and_error()
plot_errors(errors, pred_points)
