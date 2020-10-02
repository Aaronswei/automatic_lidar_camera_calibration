BUILD=build
UTEST=OFF
CMAKE_ARGS:=$(CMAKE_ARGS)

all:
	@mkdir -p $(BUILD)
	@cd $(BUILD); cmake .. -DBUILD_TEST=$(UTEST) -DCMAKE_BUILD_TYPE=Release $(CMAKE_ARGS) && $(MAKE)
	@echo -e "\n Now do 'make install' to install this package.\n"

debug:
	@mkdir -p $(BUILD)
	@cd $(BUILD); cmake .. -DBUILD_APPS=$(APPS) -DCMAKE_BUILD_TYPE=Debug $(CMAKE_ARGS) && $(MAKE)

unittest:
	@$(MAKE) all UTEST=ON
	@echo -e "\n\n Run test\n"
	@cd $(BUILD); $(MAKE) test

clean:
	@rm -rf $(BUILD)
