import urllib.request
from urllib.error import HTTPError
import sys
import os
import zipfile

# DOWNLOADER SETTINGS 
dl_DSLR = True
dl_PC = True
dl_info = True
unzip_data = True
output_directory = "D:/MOAD-data/moad-tools" 

# PRE-DEFINED OBJECT LISTS
atb1_objects = [
   "nist_atb1_board-full","nist_atb1_board-empty",
    "nist_atb1_bar-16mm","nist_atb1_bar-12mm","nist_atb1_bar-8mm","nist_atb1_bar-4mm",
    "nist_atb1_conn-bnc","nist_atb1_conn-bnc-f","nist_atb1_conn-bnc-complete",
    "nist_atb1_conn-dsub","nist_atb1_conn-dsub-f","nist_atb1_conn-dsub-complete",
    "nist_atb1_conn-ethernet","nist_atb1_conn-ethernet-f","nist_atb1_conn-ethernet-complete",
    "nist_atb1_conn-usb","nist_atb1_conn-usb-f","nist_atb1_conn-usb-complete",
    "nist_atb1_conn-wp","nist_atb1_conn-wp-f","nist_atb1_conn-wp-complete",
    "nist_atb1_gear-large","nist_atb1_gear-medium","nist_atb1_gear-small","nist_atb1_gear-plate","nist_atb1_gear-plate-complete",
    "nist_atb1_nut-m16","nist_atb1_nut-m12","nist_atb1_nut-m8","nist_atb1_nut-m4",
    "nist_atb1_rod-16mm","nist_atb1_rod-12mm","nist_atb1_rod-8mm","nist_atb1_rod-4mm"
]
atb2_objects = [
    "nist_atb2_board-full","nist_atb2_board-empty",
    "nist_atb2_timing-belt","nist_atb2_timing-pulley-small","nist_atb2_timing-pulley-large","nist_atb2_timing-spacer-small",
    "nist_atb2_timing-spacer-large","nist_atb2_timing-tensioner",
    "nist_atb2_chain","nist_atb2_chain-sprocket-small","nist_atb2_chain-sprocket-large",
    "nist_atb2_chain-sprocket-spacer-small","nist_atb2_chain-sprocket-spacer-large","nist_atb2_chain-tensioner",
    "nist_atb2_round-belt","nist_atb2_round-pulley-large","nist_atb2_round-tensioner"
]
atb3_objects = [
    "nist_atb3_board-full","nist_atb3_board-empty",
    "nist_atb3_thin-mount","nist_atb3_thin-tube","nist_atb3_thin-clip","nist_atb3_thin-cable","nist_atb3_thin-aux-connector",
    "nist_atb3_thick-mount","nist_atb3_thick-tube","nist_atb3_thick-clip","nist_atb3_thick-cable","nist_atb3_thick-ethernet-connector",
    "nist_atb3_flat-mount","nist_atb3_flat-tube","nist_atb3_flat-clip","nist_atb3_flat-cable","nist_atb3_flat-ribbon-connector",
    "nist_atb3_mount"
]
atb4_objects = [
    "nist_atb4_board-full","nist_atb4_board-empty",
    "nist_atb4_2pin","nist_atb4_2pin-housing","nist_atb4_2pin-complete",
    "nist_atb4_6pin","nist_atb4_6pin-housing","nist_atb4_6pin-complete",
    "nist_atb4_corner-post","nist_atb4_elastic-retainer"
]
atbcomp_objects = [
    "nist_atbcomp_board-full","nist_atbcomp_board-empty",
    "nist_atbcomp_2pin","nist_atb4_2pin-housing","nist_atbcomp_2pin-complete",
    "nist_atbcomp_corner-post","nist_atbcomp_elastic-retainer",
    "nist_atbcomp_round-belt","nist_atbcomp_round-pulley-small","nist_atbcomp_round-pulley-large","nist_atbcomp_round-tensioner",
    "nist_atb1_gear-small","nist_atb1_gear-large",
    "nist_atbcomp_conn-ethernet","nist_atbcomp_conn-ethernet-f","nist_atbcomp_conn-ethernet-complete",
    "nist_atbcomp_conn-bnc","nist_atbcomp_conn-bnc-f","nist_atbcomp_conn-bnc-complete",
    "nist_atb1_rod-8mm","nist_atb1_rod-16mm","nist_atb1_bar-8mm","nist_atb1_bar-12mm",
    "nist_atb1_nut-m4","nist_atbcomp_nut-m6","nist_atb1_nut-m8",
    "nist_atbcomp_nut-plate","nist_atbcomp_nut-plate-complete"
]

# USER DEFINED OBJECTS TO DOWNLOAD
# objects_to_download = atb1_objects + atb2_objects
objects_to_download = ["nist_atb1_board-full","nist_atb1_board-empty"]

def download_file(url, output_directory, filename):
    # Ensure the output directory exists
    os.makedirs(output_directory, exist_ok=True)

    # Construct the local filepath
    local_filepath = os.path.join(output_directory, filename)

    # Download the file
    try:
        urllib.request.urlretrieve(url, local_filepath)
        print(f"Downloaded: {url}\nSaved to: {local_filepath}")
    except HTTPError as e:
        print(f"Error Occurred Downloading {filename}")
        print("HTTP Error:", e.code, e.reason)

def print_config():
    print("NIST MOAD ATB DOWNLOADER CONFIG: ")

    print(f"Output Directory: {output_directory}")

    state = "True" if dl_DSLR else "False"
    print(f"[{state}]\tDownload DSLR Data (High Resolution Images)")
    state = "True" if dl_PC else "False"
    print(f"[{state}]\tDownload RealSense Data (Colored Pointclouds)")
    state = "True" if dl_info else "False"
    print(f"[{state}]\tDownload Scan Info (Text File: lux, focal length, RealSense transforms)")
    state = "True" if unzip_data else "False"
    print(f"[{state}]\tAutomatically unzip DSLR/Pointcloud Data")
    print()

if __name__ == "__main__":
    
    print_config()

    # Remove any duplicates from download list.
    download_list = []
    [download_list.append(x) for x in objects_to_download if x not in download_list]
    print(f"OBJECTS TO BE DOWNLOADED: [{len(download_list)}]\n{download_list}\n")

    # Check if user wants to continue
    user_in = input("Continue Download? (0 = No):")
    if user_in == '0':
        print("Quitting...")
        sys.exit()
    print("\n")

    # Make sure output directory exists
    if not os.path.exists(output_directory):
        print(f"Creating output directory {output_directory}...\n")
        os.makedirs(output_directory)

    # Base URL for downloads
    base_url = "https://moad-atb.s3.us-east-2.amazonaws.com/"
    c = 1
    for obj_name in download_list:
        print(f"[{c}/{len(download_list)}] NOW DOWNLOADING {obj_name}")
        if dl_info:
            file_suffix="_scan_info.txt"
            filename = f"{obj_name}{file_suffix}"
            if os.path.exists(os.path.join(output_directory,filename)):
                print(f"WARNING: \"{filename}\" already exists in output directory. Skipping download.")
            else:
                print(f"Downloading {obj_name} scan info text")
                url = f"https://moad-atb.s3.us-east-2.amazonaws.com/{obj_name}/{obj_name}{file_suffix}"
                download_file(url, output_directory, filename)
        
        if dl_DSLR:
            print(f"Downloading {obj_name} dslr image data")
            url = f"https://moad-atb.s3.us-east-2.amazonaws.com/{obj_name}/{obj_name}_DSLR.zip"
            filename = f"{obj_name}_DSLR.zip"
            download_file(url, output_directory, filename)

        if dl_PC:
            file_suffix="_realsense.zip"
            filename = f"{obj_name}{file_suffix}"
            if os.path.exists(os.path.join(output_directory,filename)):
                print(f"WARNING: \"{filename}\" already exists in output directory. Skipping {obj_name} download.")
            elif os.path.exists(os.path.join(output_directory,filename[:-4])):
                print(f"WARNING: Unzipped \"{filename[:-4]}\" folder already exists in output directory. Skipping download.")
            else:
                print(f"Downloading {obj_name} pointcloud data")
                url = f"https://moad-atb.s3.us-east-2.amazonaws.com/{obj_name}/{obj_name}{file_suffix}"
                
                download_file(url, output_directory, filename)
                # Unzip Downloaded files into a folder
                if unzip_data and os.path.exists(os.path.join(output_directory,filename)):
                    new_folder = os.path.join(output_directory,filename[:-4])
                    if not os.path.exists(new_folder):
                        os.makedirs(new_folder)
                    with zipfile.ZipFile(os.path.join(output_directory,filename), 'r') as zip_ref:
                        print(f"Extracting to {new_folder}...")
                        zip_ref.extractall(new_folder)
                    print(f"Removing .zip file ({filename})...")
                    os.remove(os.path.join(output_directory,filename))
        
        c += 1
        print("- - - - - - - -\n")
    print("Done.")