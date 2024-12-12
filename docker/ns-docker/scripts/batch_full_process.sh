#!/bin/bash

# Get the directory of the current script
base_dir="$(dirname "$(realpath "$0")")"

# Get the start time
start_time=$(date +%s)

# Initialize an error counter
error_count=0

# Define the command you want to run for each argument
script_to_run="$base_dir/full_process.sh"

# Loop through each argument passed to the script
for arg in "$@"; do
    echo "Running: $script_to_run $arg"
    $script_to_run "$arg"
    
    # Check the exit status of the command
    if [[ $? -ne 0 ]]; then
        echo "Command failed for argument: $arg (continuing)"
        ((error_count++))
    fi
done

# Get the end time in seconds since epoch
end_time=$(date +%s)

# Calculate the elapsed time in seconds
elapsed_time=$((end_time - start_time))

# Convert the elapsed time to hh:mm:ss format
elapsed_hours=$((elapsed_time / 3600))
elapsed_minutes=$(( (elapsed_time % 3600) / 60 ))
elapsed_seconds=$((elapsed_time % 60))

echo "All scans processed."
# Output the result
echo "$# scans fully processed in [$elapsed_hours:$elapsed_minutes:$elapsed_seconds]."
echo "Total errors encountered: $error_count"

