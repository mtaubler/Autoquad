#!/bin/bash
echo "Making .py into executables"
find -type f -name "*.py" -exec chmod +x \{\} \;