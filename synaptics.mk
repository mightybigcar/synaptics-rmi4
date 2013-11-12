# Makefile for Synaptics developers

# Versioning stuff.

incr = $(shell echo $(1)+1 | bc)

VER_FILE=drivers/input/rmi4/rmi_version.h

VER_MAJOR=$(shell grep RMI_VERSION_MAJOR $(VER_FILE) | head -1 | cut --delimiter=\  -f 3 | sed s/\"//g)
VER_MINOR=$(shell grep RMI_VERSION_MINOR $(VER_FILE) | head -1 | cut --delimiter=\  -f 3 | sed s/\"//g)
VER_SUBMINOR=$(shell grep RMI_VERSION_SUBMINOR $(VER_FILE) | head -1 | cut --delimiter=\  -f 3 | sed s/\"//g)
VER_EXTRA=$(shell grep RMI_EXTRA_NUMBER $(VER_FILE) | head -1| cut --delimiter=\  -f 3 | sed s/\"//g)

GIT_BRANCH=$(shell git rev-parse --abbrev-ref HEAD)
PREV_BRANCH=$(shell grep RMI_VERSION_BRANCH $(VER_FILE) | head -1 | cut --delimiter=\  -f 3 | sed s/\"//g)

ifneq ($(GIT_BRANCH),$(PREV_BRANCH))
    $(shell sed -i '/define RMI_VERSION_BRANCH/c \
#define RMI_VERSION_BRANCH \"$(GIT_BRANCH)\"' $(VER_FILE))
    $(shell sed -i '/define RMI_EXTRA_NUMBER/c \
#define RMI_EXTRA_NUMBER \"0\"' $(VER_FILE))
    VER_EXTRA=0

    ifeq ($(GIT_BRANCH),synaptics-rmi4)
        $(shell sed -i '/define RMI_EXTRA_STRING/c \
#define RMI_EXTRA_STRING \"0\"' $(VER_FILE))
    else
        EXTRA_VERSION=$(subst -,.,$(subst _,.,$(GIT_BRANCH)))
        $(shell sed -i '/define RMI_EXTRA_STRING/c \
#define RMI_EXTRA_STRING \"$(EXTRA_VERSION).0\"' $(VER_FILE))
    endif
endif


# Tarball related parameters

TARBALL:=$(GIT_BRANCH)
TAR_DATE=$(shell date --rfc-3339=date)
TAR_NAME=$(TARBALL)-$(TAR_DATE).tgz
TAR_TAG=tarball-$(TAR_DATE)


CPUS=$(shell grep processor /proc/cpuinfo | wc -l)

.PHONY: clean tarball version \
    clear-minor clear-subminor clear-extra \
    bump-major bump-minor bump-subminor bump-extra

DEFAULT: version
	@echo "Branch is $(GIT_BRANCH)."
	@echo "Cpus is $(CPUS)"

clean:
	make distclean

clear-minor:
	@sed -i '/define RMI_VERSION_MINOR/c \
#define RMI_VERSION_MINOR \"0\"' $(VER_FILE)

clear-subminor:
	@sed -i '/define RMI_VERSION_SUBMINOR/c \
#define RMI_VERSION_SUBMINOR \"0\"' $(VER_FILE)

clear-extra:
	@sed -i '/define RMI_EXTRA_NUMBER/c \
#define RMI_EXTRA_NUMBER \"0\"' $(VER_FILE)

bump-major: clear-minor clear-subminor clear-extra
	@sed -i '/define RMI_VERSION_MAJOR/c \
#define RMI_VERSION_MAJOR \"$(call incr,$(VER_MAJOR))\"' $(VER_FILE)

bump-minor: clear-subminor clear-extra
	@sed -i '/define RMI_VERSION_MINOR/c \
#define RMI_VERSION_MINOR \"$(call incr,$(VER_MINOR))\"' $(VER_FILE)

bump-subminor: clear-extra
	@sed -i '/define RMI_VERSION_SUBMINOR/c \
#define RMI_VERSION_SUBMINOR \"$(call incr,$(VER_SUBMINOR))\"' $(VER_FILE)

bump-extra:
	@sed -i '/define RMI_EXTRA_NUMBER/c \
#define RMI_EXTRA_NUMBER \"$(call incr,$(VER_EXTRA))\"' $(VER_FILE)

tarball: bump-extra
	@echo Creating $(TAR_NAME)...
	tar -czf /tmp/$(TAR_NAME) \
		--exclude drivers/input/rmi4/rmi_f30.c drivers/input/rmi4/rmi_f31.c \
		README-SYNAPTICS.txt CHANGELOG-SYNAPTICS.txt \
		Documentation/ABI/testing/*-rmi4 Documentation/input/rmi*.txt \
		drivers/input/Makefile drivers/input/Kconfig drivers/input/rmi4/Kconfig \
		drivers/input/rmi4/Makefile drivers/input/rmi4/*.[ch] include/linux/rmi.h \
		arch/arm/configs/panda_defconfig arch/arm/mach-omap2/board-omap4panda.c \
		include/linux/kconfig.h include/linux/input.h include/linux/device.h \
		include/linux/i2c.h \
		firmware/Makefile firmware/rmi4/*
	scp /tmp/$(TAR_NAME) venom:/home/cheiny/public_html/kernel.org/tarball/
	git commit -m "Updated for tarball $(TAR_NAME)" drivers/input/rmi4/rmi_version.h
	git tag $(TAR_TAG)
	git push --tags origin $(GIT_BRANCH)
	@echo Success!

version:
	@echo Major: $(VER_MAJOR)
	@echo Minor: $(VER_MINOR)
	@echo Subminor: $(VER_SUBMINOR)
	@echo Branch: $(GIT_BRANCH)
	@echo Previous branch: $(PREV_BRANCH)
	@echo Extra: $(VER_EXTRA)
	@echo Tarfile: $(TAR_NAME)
	@echo Tar tag: $(TAR_TAG)

