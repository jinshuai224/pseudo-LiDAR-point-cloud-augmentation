import os
import argparse
import hashlib
from tqdm import tqdm

def bin_file_merge(bin_file1,bin_file2,bin_merge):
    f1 = open(bin_file1,"rb")
    f2 = open(bin_file2,"rb")
    fmerge = open(bin_merge,"ab") 
    bin2_size = os.path.getsize(bin_file2)
    data = f1.read()
    fmerge.write(data)
    offset = fmerge.tell()
    data = f2.read()
    #fmerge.seek(offset1)
    fmerge.write(data)
    fmerge.close()

def main():   
    bin_dir1 = "/path_to_lidar"
    bin_dir2 = "/path_to_pseudo-lidar"
    bin_merge_dir = "/path_output"
    count = len(os.listdir(bin_dir1))
    for idx in range(count):
        bin_file1 = os.path.join(bin_dir1, '%06d.bin' % idx)
        bin_file2 = os.path.join(bin_dir2, '%06d.bin' % idx)
        bin_merge = os.path.join(bin_merge_dir, '%06d.bin' % idx)
        bin_file_merge(bin_file1,bin_file2,bin_merge)
        print("bin merge success: %06d.bin" %idx)
if __name__=='__main__':
    main()