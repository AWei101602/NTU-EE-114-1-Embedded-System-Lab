##############################################
#                README.md                   #
##############################################
## Author: Wen-Wei Chen
## Date: 2025/11/03

## Overview
Added and updated the ATPG (atpg.tcl) and fault simulation (faultsim.tcl) scripts to generate fault lists, test patterns, and simulation reports for fully and partially specified cases.

## File Structure
/R14943001_lab1/
├── atpg_fully.tcl # Generate fully specified test patterns
├── atpg_partial.tcl # Generate partially specified test patterns
├── faultsim_fully.tcl # Fault simulation (fully specified)
├── faultsim_partial.tcl # Fault simulation (partially specified)
├── s1238_LOCTDF_fully.fault
├── s1238_LOCTDF_partial.fault
├── s1238_LOCTDF_fully.stil
├── s1238_LOCTDF_partial.stil
├── report.pdf
├── README.md
├── B10901044_Lab01_docs.tgz (all the file used and generated on MobaXterm)
├── other_files/                # Contains additional supporting files
│   ├── *.log                   # ATPG or fault simulation log files
│   ├── *.csv                   # Processed coverage or histogram data exported for plotting
│   └── *.py                    # Python script used for data analysis and visualization
│   └── *.dat                   # Processed data

## Descrtiptopn
## ATPG (atpg_fully.tcl/atpg_partial.tcl)
for fully specified: 
set_atpg -fill random

for partially specified:
set_atpg -fill X

## Not required; can be commented out.
## Limit the number of decision attempts for each fault to 2000:
## set_atpg -abort_limit 2000
##
## Restrict the maximum number of generated patterns to 1000:
## set_atpg -patterns 1000
## 
## Enable high power-aware ATPG mode to reduce switching activity:
## set_atpg -power_effort High
## 
## Set the total ATPG runtime limit to 20 minutes (10 + 600 seconds):
## set_atpg -time 10 600
## 
## Generate a summary report including fault coverage and runtime:
## set_atpg -summary
## ########################################################################

## Really Iportant !!!
Set the number of capture cycles to 2 for transition delay testing (Launch-on-Capture (LOC)):
set_atpg -capture_cycles 2

Set the fault model to Transition Delay Faults:
set_faults -model Transition 

## ########################################################################
## Fault Simulation (faultsim_fully.tcl/faultsim_partial.tcl)

set num_patterns 332
for {set i 1} {$i <= $num_patterns} {incr i} {    
    ## Run fault simulation
    remove_faults -all
    add_faults -all
    set_patterns -delete
    set_patterns -external ./Netlist/s1238_LOCTDF_fully.stil
    run_fault_sim -sequential -sequential_nodrop_faults -first_pattern $i -last_pattern $i
}

set num_patterns: 332 for faultsim_fully.tcl vs. 892 for faultsim_partial.tcl

Loop from 1 to the total number of patterns: 
for {set i 1} {$i <= $num_patterns} {incr i} {  }

Remove all existing faults from the current fault list:
remove_faults -all

Add all faults in the design to the current fault list:
add_faults -all

Delete all currently loaded test patterns (fault coverage would reset to 0 because there is no pattern):
set_patterns -delete

Load external test patterns from the specified STIL file:
set_patterns -external ./Netlist/s1238_LOCTDF_fully.stil
