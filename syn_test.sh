#!/bin/bash

echo "Comparing ground truth outputs to new processor"
cd .

# Clean up previous outputs
make nuke

# Feel free to change the program name here
program=no_hazard
echo "Running $program"

# Compile and run the program
make "$program.syn.out"

echo "Comparing memory output for $program"
grep "^@@@" output/$program.syn.out > tmp_out.txt
grep "^@@@" ./syn_ground_truth/$program.syn.out > tmp_gt.txt

if diff -q tmp_out.txt tmp_gt.txt > /dev/null; then
    echo "Memory output matches for $program"
    mem_match=1
else
    echo "Memory output mismatch for $program"
    echo "Differences:"
    diff tmp_out.txt tmp_gt.txt
    mem_match=0
fi

# Clean up temporary files
rm -f tmp_out.txt tmp_gt.txt

# Print Passed or Failed
if [ $mem_match -eq 1 ]; then
    echo "$program: Passed"
else
    echo "$program: Failed"
fi
