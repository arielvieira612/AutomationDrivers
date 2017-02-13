#Compiler and Linker
CC          	:=make
ARCH			:=arm
CROSS_COMPILE	:=arm-linux-gnueabihf-
KDIR			:=/home/ariel/raspberry/linux

#The Target Binary Program
TARGET      := testebbchar

#The Directories, Source, Includes, Objects, Binary and Resources
SRCDIR      := src
INCDIR      := inc
BUILDDIR    := obj
TARGETDIR   := bin
RESDIR      := res
SRCEXT      := c
DEPEXT      := d
OBJEXT      := o
OUTEXT		:= ko
#Flags, Libraries and Includes
CFLAGS      := 
LIB         := 
INC         := -I$(INCDIR) -I/usr/local/include
INCDEP      := -I$(INCDIR)

#Exported Variables

#---------------------------------------------------------------------------------
#DO NOT EDIT BELOW THIS LINE
#---------------------------------------------------------------------------------
SOURCES     := $(shell find $(SRCDIR) -iname '*.c')
OBJECTS     := $(patsubst $(SRCDIR)/%,$(BUILDDIR)/%,$(SOURCES:.$(SRCEXT)=.$(OBJEXT)))

#Defauilt Make
all: resources $(TARGET) finish

#Remake
remake: cleaner all

#Copy Resources from Resources Directory to Target Directory
resources: directories
#	@cp $(RESDIR)/* $(TARGETDIR)/

#Make the Directories
directories:
	@mkdir -p $(TARGETDIR)
	@mkdir -p $(BUILDDIR)

#Clean only Objecst
clean:
	@$(RM) -rf $(BUILDDIR)

#Full Clean, Objects and Binaries
cleaner: clean
	@$(RM) -rf $(TARGETDIR)

#Pull in dependency info for *existing* .o files
-include $(OBJECTS:.$(OBJEXT)=.$(DEPEXT))

finish:
	$(shell mv $(shell pwd)/$(BUILDDIR)/*.ko $(shell pwd)/$(TARGETDIR))

#Link
$(TARGET):$(OBJECTS) 
	$(shell cp $(SRCDIR)/Makefile $(shell pwd)/$(BUILDDIR)/)
	$(CC) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) M=$(shell pwd)/$(BUILDDIR) modules
	
#Compile
$(BUILDDIR)/%.$(OBJEXT): $(SRCDIR)/%.$(SRCEXT)
	$(shell cp $< $(shell pwd)/$(BUILDDIR)/)
#	@mkdir -p $(dir $@)
#	$(info $$@ is [$@])
#	$(info $$< is [$<])
#	$(info $$* is [$*])

#Non-File Targets
.PHONY: all remake clean cleaner resources


