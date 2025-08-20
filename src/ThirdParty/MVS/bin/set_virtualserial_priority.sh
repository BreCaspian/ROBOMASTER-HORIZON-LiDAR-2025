#!/bin/sh
# ***************************************************************************************
#     set_virtualserial_priority.sh
# ***************************************************************************************

DisplayHelp()
{
    echo ""
    echo "NAME"
    echo "    set_virtualserial_priority.sh - Manages udev rules independently of the udev version."
    echo "DESCRIPTION"
    echo "    This script is used to create and remove udev rules from the system."
    echo "    To allow access to Virtual Serial devices, you must add a rule based on"
    echo "    the device node"
    echo ""
}

MakeRuleName()
{
    eval "$1='/etc/udev/rules.d/80-drivers-SDK-virtualserial.rules'"
}

CreateRuleFile()
{
    # Depending on the version of udev, we need to generate different type of files for the rules @#$@#$@
    VERSION_UDEV=`udevadm --version`
    if [ $VERSION_UDEV -gt 147 ]; then

> $FILENAME
cat > $1 <<__END__
ACTION=="add", KERNEL=="ttyvirserial[0-9]*", MODE="0666", GROUP="plugdev"
__END__

    else

> $FILENAME
cat > $1 <<__END__
KERNEL=="ttyvirserial[0-9]*", MODE="0666"
__END__

    fi
}

Add()
{
    local FILENAME

    if [ ! -d "/etc/udev/rules.d" ]; then
        echo "Unable to find the folder that contains the rules (/etc/udev/rules.d)."
        exit 3
    fi
    if ! which udevadm >/dev/null 2>&1; then
        echo "Unable to find the udevadm tool to retrieve the current version of udev that is"
        echo "installed on this distribution." 
        echo "You must add the rule manually."
        exit 3
    fi

    MakeRuleName FILENAME
    echo "Adding rules for virtual serial device."

    # Create the new rule file
    CreateRuleFile "$FILENAME"
    # Set the permission to the files
    chmod u=rw,g=r,o=r $FILENAME

    echo "The $FILENAME rule has been created."
}

# Default variables
USER_ID=`id -u`

# Check required priviledge
if [ "$USER_ID" != "0" ]; then
  echo "Can only run this script as superuser (root account)."
  exit 1
fi


ACTION="add"

# Now finally, we can process the information...
echo ""
if [ "$ACTION" = "add" ]; then
    Add
fi
echo ""



