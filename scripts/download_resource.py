# download zipped resource folder

import sys
import urllib.request

file_url         = "https://drive.google.com/uc?export=download&id=15xh8dKaCpgNTunFZWeaeBhE6Dio3YPgR"
target_directory = sys.argv[1]
zip_file_path    = "./temp_Resource.zip"

def progress_reporter(num_chunks_read, chunk_size, total_size):
    read_so_far = num_chunks_read * chunk_size
    print("\r - Downloaded: %d MB -" % (read_so_far / (1 << 20)), flush = True, end = "")

print("Downloading resource file %s..." % zip_file_path)
urllib.request.urlretrieve(file_url, zip_file_path, progress_reporter)
print("\nDownload complete.")

# extract zipped resource folder

print("Extracting file...")

import zipfile
zip_file = zipfile.ZipFile(zip_file_path, "r")
zip_file.extractall(target_directory)
zip_file.close()

# delete zipped resource file

print("Deletes temporary file %s." % zip_file_path)

import os
os.remove(zip_file_path)