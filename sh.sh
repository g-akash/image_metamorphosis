#!/bin/bash
./morph data/pisaa.jpg data/pisab.jpg data/pisap.txt 0 pisa0.0.png
for i in $(seq 1 10); do 
RESULT=$(echo "$i/10" | bc -l);
./morph data/pisaa.jpg data/pisab.jpg data/pisap.txt $RESULT pisa0.$i.png; 
done