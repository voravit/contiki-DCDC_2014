CONTIKI_PROJECT = bidcdc-coap
TARGET = bi-dcdc
OBJDIR = obj_$(TARGET)
CONTIKI = ../..

all: CHECK_OBJECTDIR $(CONTIKI_PROJECT)

CONTIKI_TARGET_SOURCEFILES += bang.c control.c adc.c   

CHECK_OBJECTDIR:
	if [ ! -d "$(OBJDIR)" ]; then mkdir $(OBJDIR);	fi

UIP_CONF_IPV6=1
UIP_CONF_RPL=1
#If BOARD_2013 is defined then we have the platform built for then new board built on 2013 otherwise the old board i.e 2012
CFLAGS += -DWITH_COAP=13 -DUIP_CONF_TCP=0 -DREST=coap_rest_implementation -DBOARD_2013
APPS += er-coap-13 erbium


include $(CONTIKI)/Makefile.include

run: $(CONTIKI_PROJECT).$(TARGET)
	$(OBJCOPY) -O ihex $(CONTIKI_PROJECT).$(TARGET) $(CONTIKI_PROJECT).ihex
	sudo lpc21isp -hex $(CONTIKI_PROJECT).ihex /dev/ttyUSB0 115200 12000

clean:
	rm -rf $(OBJDIR) *.$(TARGET) *.a *.map *.ihex
