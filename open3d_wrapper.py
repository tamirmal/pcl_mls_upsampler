import open3d as o3d
import argparse
import os
import subprocess


def upsampler(in_o3d):
    out_path = './tmp_out.pcd'
    in_path = './tmp_in.pcd'
    o3d.io.write_point_cloud(in_path, in_o3d)

    if os.path.isfile(in_path):
        print(f"input file {in_path} exists - was not cleared properly!")
        assert 0
    if os.path.isfile(out_path):
        print(f"output file {out_path} exists - was not cleared properly!")
        assert 0

    subprocess.check_call(['./upsampler', in_path, out_path])
    upsampled_pcd = o3d.io.read_point_cloud(out_path)

    os.remove(out_path)
    os.remove(in_path)

    return upsampled_pcd


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()

    upsampler(args.input, args.output)


if __name__ == "__main__":
    main()
