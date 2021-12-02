KVERS = $(shell uname -r)
obj-m :=globalmem.o
# 如果含有多个文件:
#obj -m :=hello.o
# hello-objs :=file1.o file2.o

build :kernel_modules

kernel_modules:
		make -C /lib/modules/$(KVERS)/build M=$(CURDIR) modules  
		
clean: 
		make -C /lib/modules/$(KVERS)/build M=$(CURDIR) clean