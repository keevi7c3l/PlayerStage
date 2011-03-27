#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc
CCC=g++
CXX=g++
FC=
AS=as

# Macros
CND_PLATFORM=GNU-Linux-x86
CND_CONF=Debug
CND_DISTDIR=dist

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=build/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/PathPlanner.o \
	${OBJECTDIR}/main.o \
	${OBJECTDIR}/PlayerWrapper.o \
	${OBJECTDIR}/LaserReader.o \
	${OBJECTDIR}/Mapper.o


# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS=`pkg-config --cflags playerc` `pkg-config --libs playerc` -std=c++0x -lpthread -L/usr/local/lib  -lm -lcv -lhighgui -lcxcore -ljpeg 
CXXFLAGS=`pkg-config --cflags playerc` `pkg-config --libs playerc` -std=c++0x -lpthread -L/usr/local/lib  -lm -lcv -lhighgui -lcxcore -ljpeg 

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-Debug.mk dist/Debug/GNU-Linux-x86/playerstage_classed

dist/Debug/GNU-Linux-x86/playerstage_classed: ${OBJECTFILES}
	${MKDIR} -p dist/Debug/GNU-Linux-x86
	${LINK.cc} -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/playerstage_classed ${OBJECTFILES} ${LDLIBSOPTIONS} 

${OBJECTDIR}/PathPlanner.o: PathPlanner.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/PathPlanner.o PathPlanner.cpp

${OBJECTDIR}/main.o: main.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/main.o main.cpp

${OBJECTDIR}/PlayerWrapper.o: PlayerWrapper.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/PlayerWrapper.o PlayerWrapper.cpp

${OBJECTDIR}/LaserReader.o: LaserReader.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/LaserReader.o LaserReader.cpp

${OBJECTDIR}/Mapper.o: Mapper.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -MMD -MP -MF $@.d -o ${OBJECTDIR}/Mapper.o Mapper.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/Debug
	${RM} dist/Debug/GNU-Linux-x86/playerstage_classed

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
