cmd_/opt/MVS/driver/gige/TransportLayer/modules.order := {   echo /opt/MVS/driver/gige/TransportLayer/gevfilter.ko; :; } | awk '!x[$$0]++' - > /opt/MVS/driver/gige/TransportLayer/modules.order
