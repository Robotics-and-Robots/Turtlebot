BIN_NAME  :=driver
SRC_DIR   :=src
OUT_DIR   :=output

CPP_FLAGS :=-Iinclude -o $(OUT_DIR)/$(BIN_NAME).exe
LD_FLAGS  :=-lpthread


SRC_FILES :=$(SRC_DIR)/Buffer.cpp $(SRC_DIR)/Driver.cpp

all:
	g++ $(CPP_FLAGS) $(SRC_FILES) $(LD_FLAGS)
	./$(OUT_DIR)/$(BIN_NAME).exe
