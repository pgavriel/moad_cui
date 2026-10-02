import numpy as np
import sys
import matplotlib.pyplot as plt

def read_colmap_array(path):
    with open(path, "rb") as f:
        # header: "width&height&channels&"
        header = b""
        while header.count(b"&") < 3:
            header += f.read(1)
        w, h, c = map(int, header.decode().split("&")[:3])
        data = np.fromfile(f, np.float32)
    return np.transpose(data.reshape((w, h, c), order="F"), (1, 0, 2)).squeeze()

depth = read_colmap_array(sys.argv[1])
valid = depth > 0
print(f"shape: {depth.shape}")
print(f"valid pixels: {valid.mean()*100:.1f}%")
if valid.any():
    v = depth[valid]
    print(f"depth range: min={v.min():.4f}  p5={np.percentile(v,5):.4f}  "
          f"median={np.median(v):.4f}  p95={np.percentile(v,95):.4f}  max={v.max():.4f}")
    plt.imshow(np.clip(depth, *np.percentile(depth[valid], [5, 95])))
    lo, hi = np.percentile(depth[valid], [5, 95])
    fig, ax = plt.subplots(1, 3, figsize=(18, 5))
    rgb_path = "/home/csrobot/MOAD_DATA/ex2_006/pose-a/images_4/frame_00001.jpg"
    ax[0].imshow(plt.imread(rgb_path))
    ax[1].imshow(np.where(valid, np.clip(depth, lo, hi), np.nan)); ax[1].set_title("depth (clipped)")
    ax[2].imshow(depth > hi); ax[2].set_title("depth > p95 (suspected background)")
    plt.show()