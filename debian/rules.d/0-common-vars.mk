# Get some version info
stub=linux

release := $(shell sed -n '1s/^.*(\(.*\)-.*).*$$/\1/p' debian/changelog)
revisions := $(shell sed -n 's/^$(stub)\ .*($(release)-\(.*\)).*$$/\1/p' debian/changelog | tac)
revision ?= $(word $(words $(revisions)),$(revisions))
prev_revisions := $(filter-out $(revision),0.0 $(revisions))
prev_revision := $(word $(words $(prev_revisions)),$(prev_revisions))

# This is an internally used mechanism for the daily kernel builds. It
# creates packages who's ABI is suffixed with a minimal representation of
# the current git HEAD sha. If .git/HEAD is not present, then it uses the
# uuidgen program,
#
# AUTOBUILD can also be used by anyone wanting to build a custom kernel
# image, or rebuild the entire set of Ubuntu packages using custom patches
# or configs.
AUTOBUILD=

#
# This is a way to support some external variables. A good example is
# a local setup for ccache and distcc See LOCAL_ENV_CC and
# LOCAL_ENV_DISTCC_HOSTS in the definition of kmake.
# For example:
#      LOCAL_ENV_CC="ccache distcc"
#      LOCAL_ENV_DISTCC_HOSTS="localhost 10.0.2.5 10.0.2.221"
#
-include $(CURDIR)/../.intrepid-env

ifneq ($(AUTOBUILD),)
skipabi		= true
skipmodule	= true
skipdbg		= true
gitver=$(shell if test -f .git/HEAD; then cat .git/HEAD; else uuidgen; fi)
gitverpre=$(shell echo $(gitver) | cut -b -3)
gitverpost=$(shell echo $(gitver) | cut -b 38-40)
abi_suffix = -$(gitverpre)$(gitverpost)
endif

ifneq ($(NOKERNLOG),)
ubuntu_log_opts += --no-kern-log
endif
ifneq ($(PRINTSHAS),)
ubuntu_log_opts += --print-shas
endif

ifeq ($(wildcard /CurrentlyBuilding),)
skipdbg=true
endif

abinum		:= $(shell echo $(revision) | sed -e 's/\..*//')$(abi_suffix)
prev_abinum	:= $(shell echo $(prev_revision) | sed -e 's/\..*//')$(abi_suffix)

abi_release	:= $(release)-$(abinum)

# We force the sublevel to be exactly what we want. The actual source may
# be an in development git tree. We want to force it here instead of
# committing changes to the top level Makefile
SUBLEVEL	:= $(shell echo $(release) | awk -F. '{print $$3}')

arch		:= $(shell dpkg-architecture -qDEB_HOST_ARCH)
abidir		:= $(CURDIR)/debian/abi/$(release)-$(revision)/$(arch)
prev_abidir	:= $(CURDIR)/debian/abi/$(release)-$(prev_revision)/$(arch)
confdir		:= $(CURDIR)/debian/config/$(arch)
builddir	:= $(CURDIR)/debian/build
stampdir	:= $(CURDIR)/debian/stamps

# Support parallel=<n> in DEB_BUILD_OPTIONS (see #209008)
COMMA=,
DEB_BUILD_OPTIONS_PARA = $(subst parallel=,,$(filter parallel=%,$(subst $(COMMA), ,$(DEB_BUILD_OPTIONS))))
ifneq (,$(DEB_BUILD_OPTIONS_PARA))
  CONCURRENCY_LEVEL := $(DEB_BUILD_OPTIONS_PARA)
endif

ifeq ($(CONCURRENCY_LEVEL),)
  # Check the environment
  CONCURRENCY_LEVEL := $(shell echo $$CONCURRENCY_LEVEL)
  # No? Check if this is on a buildd
  ifeq ($(CONCURRENCY_LEVEL),)
    ifeq ($(wildcard /CurrentlyBuilding),)
      CONCURRENCY_LEVEL := $(shell expr `getconf _NPROCESSORS_ONLN` \* 2)
    endif
  endif
  # Oh hell, give 'em one
  ifeq ($(CONCURRENCY_LEVEL),)
    CONCURRENCY_LEVEL := 1
  endif
endif

conc_level		= -j$(CONCURRENCY_LEVEL)

# target_flavour is filled in for each step
kmake = make ARCH=$(build_arch) EXTRAVERSION=-$(abinum)-$(target_flavour) \
	CONFIG_DEBUG_SECTION_MISMATCH=y SUBLEVEL=$(SUBLEVEL)
ifneq ($(LOCAL_ENV_CC),)
kmake += CC=$(LOCAL_ENV_CC) DISTCC_HOSTS=$(LOCAL_ENV_DISTCC_HOSTS)
endif
