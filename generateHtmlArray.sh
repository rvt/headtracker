#!/bin/bash

cd web
cat *.js | gzip -9 > jscript.js.gz
xxd --include jscript.js.gz > ../src/jscript.generated.h
rm jscript.js.gz

for f in *.html
do
    cat $f > "${f}.nt"
    echo -ne "\0" >> "${f}.nt"
    xxd -i "${f}.nt" > ../src/"${f/\./_}".generated.h
done
rm -rf *.nt
