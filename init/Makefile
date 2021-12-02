KVERS = $(shell uname -r)
obj-m :=globalmem.o
# 如果含有多个文件:
#obj -m :=hello.o
# hello-objs :=file1.o file2.o

build :kernel_modules

kernel_modules:
		make -C /lib/modules/$(KVERS)/build M=$(CURDIR) modules  
		# -C 表示采用编译的源代码位置（模块源代码头文件包含的位置）
		# M表示所编译模块的源代码目录 
		
clean: 
		make -C /lib/modules/$(KVERS)/build M=$(CURDIR) clean