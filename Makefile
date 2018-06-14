icozctl: icozctl.cc
	gcc -o icozctl -Wall -Os icozctl.cc -lwiringPi -lyaml-cpp -lrt -lstdc++

install: icozctl
	sudo install icozctl /usr/local/bin/
	sudo chmod u+s /usr/local/bin/icozctl

uninstall:
	sudo rm -f /usr/local/bin/icozctl

clean:
	rm -f icozctl

.PHONY: install clean

