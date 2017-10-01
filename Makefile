obj-m  := piadcsnd.o 


# piadcsnd-y := piadcsnd.o 

# pcsp_lib.o 


all:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) modules
clean:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) clean

