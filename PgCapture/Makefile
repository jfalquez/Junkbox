# makefile

CC = g++
OUTPUTNAME = PgCapture
INCLUDE = -I/usr/include/flycapture -I/home/rpg/Code/Pangolin -I/home/rpg/Code/SceneGraph -I/home/rpg/Code/Builds/Pangolin -I/home/rpg/Code/Builds/SceneGraph -I/usr/include/eigen3
LIBS = -L/usr/lib -lflycapture -L/home/rpg/Code/Builds/Pangolin/pangolin -lpangolin -L/home/rpg/Code/Builds/SceneGraph/SceneGraph -lSceneGraph -lglut

OUTDIR = ./

OBJS = PgCapture.o

${OUTPUTNAME}: ${OBJS}
	${CC} -o ${OUTPUTNAME} ${OBJS} ${LIBS} ${COMMON_LIBS} 
	strip --strip-unneeded ${OUTPUTNAME}
#	mv ${OUTPUTNAME} ${OUTDIR}

%.o: %.cpp
	${CC} ${CFLAGS} ${INCLUDE} -Wall -c $*.cpp
	
clean_obj:
	rm -f ${OBJS}	@echo "all cleaned up!"

clean:
	rm -f ${OUTDIR}/${OUTPUTNAME} ${OBJS}	@echo "all cleaned up!"
