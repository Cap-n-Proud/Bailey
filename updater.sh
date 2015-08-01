#!/bin/bash

#Parse data from the json config file
installPath = "$(ruby -rjson -e 'j = JSON.parse(File.read("server/app/config.json")); puts j["server"]["installPath"]')"
installedVersion="$(ruby -rjson -e 'j = JSON.parse(File.read("server/app/config.json")); puts j["server"]["version"]')"

installPath="/home/paolo/pfnegrini@gmail.com/Git/Bailey"

#Check local git version
latestVersion="$(git -C $installPath tag | sort -n | tail -1)" 


repository="https://github.com/pfnegrini/Bailey/archive/"$latestVersion".tar.gz"

echo current version installedVersion latest $latestVersion
if [ "$latestVersion" > "$installedVersion" ]; 
then
    
    echo Need update
    mkdir updater
    echo downloading repository $repository
    wget  $repository -P updater
    #rm -rf $installPath
    tar -xvf updater/$latestVersion.tar.gz -C destination_folder --strip 1
    #rm -rf updater
    
    else
    echo Software updated
fi
