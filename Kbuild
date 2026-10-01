LINUXINCLUDE   += -I$(NFC_ROOT)/include/uapi/linux/nfc/

obj-m += stm_nfc_i3c.o

ccflags-y := $(call cc-option,-Wno-misleading-indentation)
stm_nfc_i3c-y :=  nfc/st21nfc_i3c.o
