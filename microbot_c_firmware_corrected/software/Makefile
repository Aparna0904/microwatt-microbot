CC ?= gcc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -pedantic -O2
CPPFLAGS += -Iinclude -Itests
SRC = src/platform.c src/platform_host.c src/servo_driver.c src/motor_driver.c \
      src/hc_sr04_driver.c src/obstacle_detection.c src/navigation.c \
      src/uart_command.c src/scenario.c
TEST_SRC = tests/test_main.c
.PHONY: all test scenarios clean
all: test scenarios
test:
	$(CC) $(CFLAGS) $(CPPFLAGS) -DMB_HOST_BUILD=1 $(SRC) $(TEST_SRC) -o microbot_tests
	./microbot_tests
scenarios:
	$(CC) $(CFLAGS) $(CPPFLAGS) -DMB_HOST_BUILD=1 -DMB_SCENARIO=1 $(SRC) src/main.c -o microbot_scenario1
	$(CC) $(CFLAGS) $(CPPFLAGS) -DMB_HOST_BUILD=1 -DMB_SCENARIO=2 $(SRC) src/main.c -o microbot_scenario2
	$(CC) $(CFLAGS) $(CPPFLAGS) -DMB_HOST_BUILD=1 -DMB_SCENARIO=3 $(SRC) src/main.c -o microbot_scenario3
clean:
	rm -f microbot_tests microbot_scenario1 microbot_scenario2 microbot_scenario3
