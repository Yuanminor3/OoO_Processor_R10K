
cd /user/stud/fall24/yj2848/EE4824/Final_proj_test_our_code

# Feel free to change the program name here
program=dft


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
