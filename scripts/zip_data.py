import os
import shutil
import time

def format_time(seconds):
    hours = seconds // 3600
    minutes = (seconds % 3600) // 60
    seconds = seconds % 60
    return f"{hours:02d}:{minutes:02d}:{seconds:02d}"

# Directory to search for uncompressed data folders
root_directory = "D:\MOAD"
# Directory to create compressed data folders
output_directory = "D:\MOAD-data"
# Prefix
prefix = "nist_atb3_"

archive_format="zip" #"gztar"
print(shutil.get_archive_formats())

print(f"Root Directory: {root_directory}")
subfolders = [folder for folder in os.listdir(root_directory) if os.path.isdir(os.path.join(root_directory, folder)) and folder.startswith(prefix)]

subfolders = ["nist_atb1_conn-usb-complete",
"nist_atb1_conn-dsub-complete",
"nist_atb4_6pin-complete",
"nist_atb4_2pin-complete",
"nist_atb3_thick-cable",
"nist_atb3_flat-cable",
"nist_atb1_conn-wp-complete",
"nist_atb1_gear-plate-complete",
"nist_atb1_conn-bnc-complete",
"nist_atb1_conn-ethernet-complete"]
print(f"{len(subfolders)} Subfolders Found:\n{subfolders}")

should_run = input("Run Compression? (0 = No): ")
if should_run == 0:
    run = False
else:
    run = True

start_time = time.time()
count = 0
for subfolder in subfolders:
    if (not run): break
    loop_start_time = time.time()
    output_subfolder = os.path.join(output_directory, subfolder)
    if not os.path.exists(output_subfolder):
        os.makedirs(output_subfolder)
        count += 1
        print(f"Zipping {count}/{len(subfolders)}")
        print(f"Created folder: {output_subfolder}")

        dslr_folder = os.path.join(root_directory, subfolder, "DSLR")
        realsense_folder = os.path.join(root_directory, subfolder, "realsense")
        
        # Copy the text files to the output subfolder
        input_folder = os.path.join(root_directory, subfolder)
        for file in os.listdir(input_folder):
            if file.endswith(".txt"):
                input_file = os.path.join(input_folder, file)
                output_file = os.path.join(output_subfolder, file)
                shutil.copy(input_file, output_file)
                print(f"Copied {output_file}")

        if os.path.exists(dslr_folder):
            print(f"{archive_format}-ing DSLR folder...")
            shutil.make_archive(os.path.join(output_subfolder, subfolder+"_DSLR"), archive_format, dslr_folder)
            print(f"Compressed DSLR folder in: {output_subfolder}")
        else:
            print(f"DSLR folder not found in: {root_directory}/{subfolder}")

        if os.path.exists(realsense_folder):
            print(f"{archive_format}-ing realsense folder...")
            shutil.make_archive(os.path.join(output_subfolder, subfolder+"_realsense"), archive_format, realsense_folder)
            print(f"Compressed Realsense folder in: {output_subfolder}")
        else:
            print(f"Realsense folder not found in: {root_directory}/{subfolder}")
    else:
        print(f"\nFolder already exists: {output_subfolder}\n")

    total_time = time.time() - loop_start_time
    print(f"Total Time: {format_time(int(total_time))}")

    #break
total_time = time.time() - start_time
print(f"Total Time: {format_time(int(total_time))}")