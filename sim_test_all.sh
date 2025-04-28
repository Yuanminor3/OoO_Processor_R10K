#!/bin/bash

echo "Comparing sim ground truth outputs to new processor"
cd .

# Clean up previous outputs
make nuke

pass_count=0
total_count=0
fail_list=()

# Create or clear the result file
result_file="sim_all_results.txt"
> "$result_file"

for source_file in programs/*.s programs/*.c; do
    if [ "$source_file" = "programs/crt.s" ]; then
        continue
    fi

    total_count=$((total_count + 1))
    program=$(echo "$source_file" | cut -d '.' -f1 | cut -d '/' -f2)
    echo "Running $program" | tee -a "$result_file"

    # Compile and run the program
    make "$program.out"

    echo "Comparing memory output for $program" | tee -a "$result_file"
    grep "^@@@" output/$program.out > tmp_out.txt
    grep "^@@@" ./sim_ground_truth/$program.out > tmp_gt.txt

    if diff -q tmp_out.txt tmp_gt.txt > /dev/null; then
        echo "Memory output matches for $program" | tee -a "$result_file"
        mem_match=1
    else
        echo "Memory output mismatch for $program" | tee -a "$result_file"
        echo "Differences:" | tee -a "$result_file"
        diff tmp_out.txt tmp_gt.txt | tee -a "$result_file"
        mem_match=0
    fi

# Print Passed or Failed
    if [ $mem_match -eq 1 ]; then
        echo "$program: Passed" | tee -a "$result_file"
        pass_count=$((pass_count + 1))
    else
        echo "$program: Failed" | tee -a "$result_file"
        fail_list+=("$program")
    fi
done

# Summary
echo "======================" | tee -a "$result_file"
echo "Sim Total Passed: $pass_count out of $total_count" | tee -a "$result_file"

if [ $pass_count -eq $total_count ]; then
    echo "ALL PASSED!" | tee -a "$result_file"
else
    echo "FAILED Programs:" | tee -a "$result_file"
    for fail_prog in "${fail_list[@]}"; do
        echo "$fail_prog" | tee -a "$result_file"
    done
fi
