#!/bin/bash

# Function to display usage information
usage() {
    echo "Usage: $0 -u <username> -i <ip_address> -p <password>"
    exit 1
}

# Parse command-line options
while getopts ":u:i:p:" opt; do
    case ${opt} in
        u ) username=$OPTARG ;;
        i ) ip_address=$OPTARG ;;
        p ) password=$OPTARG ;;
        \? ) usage ;;
        : ) echo "Error: Option -$OPTARG requires an argument." >&2; usage ;;
    esac
done
shift $((OPTIND -1))

# Check if all required options are provided
if [[ -z "$username" || -z "$ip_address" || -z "$password" ]]; then
    usage
fi

# Define the commands to be executed on the remote server
remote_commands=$(cat << 'EOF'
cd aimotion_f1tenth_system
source startup_framework.sh
EOF
)

# Use SSH to execute the commands on the remote server
sshpass -p "$password" ssh -o StrictHostKeyChecking=no $username@$ip_address "$remote_commands"
