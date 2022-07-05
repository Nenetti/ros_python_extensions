#!/bin/bash

python3 -m pytest
exit_code="${?}"
echo ${exit_code}