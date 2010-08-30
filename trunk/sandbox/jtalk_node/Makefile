super-all: aques all

include $(shell rospack find mk)/cmake.mk

aques: aquestalk2/libAquesTalk2.so aquestalk2/libAquesTalk2.so.1

aquestalk2/libAquesTalk2.so.1.0: aqtk2_lnx32_100125.tgz
	tar zxf $<

aquestalk2/libAquesTalk2.so.1: aquestalk2/libAquesTalk2.so.1.0
	(cd aquestalk2; ln -sf libAquesTalk2.so.1.0 libAquesTalk2.so.1)

aquestalk2/libAquesTalk2.so: aquestalk2/libAquesTalk2.so.1.0
	(cd aquestalk2; ln -sf libAquesTalk2.so.1.0 libAquesTalk2.so)

aqtk2_lnx32_100125.tgz:
	wget http://www.a-quest.com/download/package/aqtk2_lnx32_100125.tgz

clean-aques:
	rm -rf aqtk2_lnx32_100125.tgz aquestalk2
