#pragma once
/*
    ScriptRunner.h
    --------------
    Wrappers for calling external Python scripts from the MOAD pipeline.
    Each function is self-contained: all required state is passed explicitly
    as parameters so there are no dependencies on MOADCui globals.

    Scripts called:
        scripts/create_object_info.py   — create_obj_info_json()
        scripts/transform_generator.py  — generate_transforms()
        scripts/filecount_test.py       — run_filecount_check()
*/

#include <string>

/*
    Create the object_info.json template file for the given object.

    Args:
        output_dir  : root data output directory (e.g. config["output_dir"])
        object_name : name of the object being scanned
*/
void create_obj_info_json(const std::string& output_dir, const std::string& object_name);

/*
    Generate a transforms.json file for NeRF training by calling
    scripts/transform_generator.py.

    Args:
        degree_inc  : degrees per turntable move
        num_moves   : total number of moves in the scan
        curr_pose   : current pose letter (e.g. 'a', 'b', ...)
*/
bool generate_transforms(int degree_inc, int num_moves, char curr_pose);

/*
    Run scripts/filecount_test.py to verify and optionally create
    downscaled image sets after a scan.

    Args:
        scan_folder : path to the current object's data folder

    Returns:
        true  if the script was run
        false if filecount_testing is disabled in config
*/
bool run_filecount_check(const std::string& scan_folder);

/* ========================================================================
    SCENE REPLICA SUB-MODULE
======================================================================== */
/*
Scene Replica Live Viewer Script
*/
bool run_replica_live_view();
bool replica_generate_annotations();

/*
Scene Viewer Script ?
*/
