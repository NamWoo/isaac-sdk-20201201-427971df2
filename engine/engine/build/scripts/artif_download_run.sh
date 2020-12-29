#!/bin/bash
#####################################################################################
# Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
# This script pulls the necessary files for running the tests on Jetson from the Artifactory and the runs all the tests.
# After tests are done running it reports the results back.


curl -$1:$2 -O $3
tar -xvf $4
rm -rf $4

rm -rf jetson_testlog
mkdir jetson_testlog

BUILD_TESTLOG=${PWD}/jetson_testlog

for WS in "engine" "sdk";
do

rm -rf jetson_artifactory

tar -xvf ${WS}.tar
rm ${WS}.tar
pushd jetson_artifactory

WS_ARTIFACTORY=$PWD

for dir in */; do cd "$dir"; done
rm -rf jetson_testlog
rm -rf aborted_testlog
rm -rf aborted_tests.xml
mkdir jetson_testlog
rm -rf jetson-test-out.txt
export LD_LIBRARY_PATH=$PWD/test_dep
total_aborted=0

function xml_string_manipulation ()
{
    sed -i "s/insert_test_suite/$2/g" $1
    sed -i "s/insert_test_case/$3/g" $1
    sed -i 's/\x1b\[[0-9;]*m//g' $1
    cp $1 $PWD/jetson_testlog
    rm -rf $1
}

#Load the error message template, in case a test gets aborted
ERR1='' read -r -d '' error_template_1 <<"EOF"
<?xml version="1.0" encoding="UTF-8"?>
<testsuites tests="1" failures="1" disabled="0" errors="0" time="0" name="AllTests">
  <testsuite name="insert_test_suite" tests="1" failures="1" disabled="0" errors="0" time="0">
    <testcase name="insert_test_case" status="run" time="0" classname="insert_test_suite">
      <failure message="TEST ABORTED. SEE STACK TRACE FOR TERMINAL OUTPUT:" type=""><![CDATA[
EOF
ERR2='' read -r -d '' error_template_2 <<"EOF"
]]></failure>
    </testcase>
  </testsuite>
</testsuites>
EOF
ERR3='' read -r -d '' error_template_3 <<"EOF"
<?xml version="1.0" encoding="UTF-8"?>
<testsuites tests="1" failures="1" disabled="0" errors="0" time="0" name="AllTests">
  <testsuite name="insert_test_suite" tests="1" failures="1" disabled="0" errors="0" time="0">
    <testcase name="insert_test_case" status="run" time="0" classname="insert_test_suite">
      <failure message="NO TESTS WERE COLLECTED. SEE STACK TRACE FOR TERMINAL OUTPUT:" type=""><![CDATA[
EOF

WS_TEST_LOG=$PWD/jetson_testlog
while read line; do
    # addresses of the test directories are stored in the $line variable now
    echo "================================================================================"
    echo "RUNNING $line"
    echo "================================================================================"
    # list the tests here and then run them individually --gtest_list_tests
    ${PWD}/$line --gtest_list_tests |& tee test_cases
    test_first=$(head -n 1 test_cases)
    if [ "$test_first" == "Running main() from gtest_main.cc" ]; then
        tail -n +2 "test_cases" > "test_cases.tmp" && mv "test_cases.tmp" "test_cases"
    fi
    while read test_case_line; do
        last_char=${test_case_line: -1}
        if [ "$last_char" == "." ]; then
            test_suite=${test_case_line}
        else
            test_case="${test_suite}${test_case_line}"
            xml_file="${WS}_${test_case}.xml"
            counter_before=$(ls -1q ${WS_TEST_LOG} | wc -l)
            timeout 120 ./$line --gtest_color=no --gtest_filter="$test_case" --gtest_output="xml:${WS_TEST_LOG}/" >gtest_std_output 2>gtest_error_msg
            gtest_exit_code=$?
            counter_after=$(ls -1q ${WS_TEST_LOG} | wc -l)
            if [ "$counter_before" -eq "$counter_after" ]; then
                total_aborted=$((total_aborted + 1))
                msg1=$(<gtest_std_output)
                msg2=$(<gtest_error_msg)
                echo -e "${error_template_1}\n${msg1}\n${msg2}\n${error_template_2}" > ${xml_file}
                xml_string_manipulation ${xml_file} ${test_suite::-1} $test_case_line
            fi
        fi
    done <<<$(cat $PWD/test_cases)
done <<<$(cat $PWD/all_tests_formatted)

while read line; do
    # addresses of the test directories are stored in the $line variable now
    echo "================================================================================"
    echo "RUNNING PYTEST $line"
    echo "================================================================================"
    pytest_name="${line##*/}"
    xml_name="${WS}_${pytest_name}.xml"
    PYTHONPATH=$PWD:$PWD/packages/pyalice python3 -m pytest --junitxml=${WS_TEST_LOG}/${xml_name} $line.py >std_output 2>error_msg
    exit_code=$?
    output=$(<std_output)
    echo $output
    if [ $exit_code -eq 4 ]; then
        msg=$(<error_msg)
        echo -e "${error_template_1}\n${msg}\n${error_template_2}" > ${xml_name}
        xml_string_manipulation ${xml_name} $pytest_name pytest
    fi
    if [ $exit_code -eq 5 ]; then
        msg=$(<std_output)
        echo -e "${error_template_3}\n{$msg}\n${error_template_2}" > ${xml_name}
        xml_string_manipulation ${xml_name} $pytest_name pytest
    fi
    rm -rf std_output
    rm -rf error_msg
done <<<$(cat $PWD/python_tests_formatted)

# change all the classnames here to Jetson.<classname>, to seperate from bazel tests
for file in $PWD/jetson_testlog/*
do
    sed -i 's/classname="/&Jetson\./g' $file
done

# copy test reuslts to jenkins workspace so the pipeline can access them
echo "Moving Test Log from ${WS_TEST_LOG} to ${BUILD_TESTLOG}"
mkdir "${BUILD_TESTLOG}/${WS}"
cp -r "${WS_TEST_LOG}"/* "${BUILD_TESTLOG}/${WS}/"

popd

done
