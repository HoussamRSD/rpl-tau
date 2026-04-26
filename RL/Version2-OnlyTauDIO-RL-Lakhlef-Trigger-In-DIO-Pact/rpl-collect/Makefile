CONTIKI = ../../..
APPS = powertrace collect-view
CONTIKI_PROJECT = udp-sender udp-sink
PROJECT_SOURCEFILES += collect-common.c

# Note: rpl-of-tau.c has been added to ../rpl/ and will be automatically
# compiled as part of the RPL module. No explicit Makefile changes needed.

# RL Q-Learning agent (lives in ../rpl/ alongside rpl-dag.c)
PROJECT_SOURCEFILES += rpl-rl-agent.c
VPATH += ../rpl

CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"

ifeq ($(MAKE_WITH_NON_STORING),1)
CFLAGS += -DWITH_NON_STORING=1
endif

ifdef PERIOD
CFLAGS=-DPERIOD=$(PERIOD)
endif

all: $(CONTIKI_PROJECT)

CONTIKI_WITH_IPV6 = 1
include $(CONTIKI)/Makefile.include
