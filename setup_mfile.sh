#! /usr/bin/env bash
sudo apt-get install tcl8.5 tk8.5 expect 
wget http://www.sax.de/~joerg/mfile/mfile.tar.gz
tar -xzf mfile.tar.gz 
rm mfile.tar.gz
cd mfile
echo 'export MFILE_HOME="." ; wish mfile.tcl' > run.sh
chmod +x run.sh
./run.sh
