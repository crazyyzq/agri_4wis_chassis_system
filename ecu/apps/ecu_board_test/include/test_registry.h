#ifndef ECU_TEST_REGISTRY_H
#define ECU_TEST_REGISTRY_H
#include <stddef.h>
#include "test_runner.h"
const test_descriptor_t *test_registry_all(size_t *count);
const test_descriptor_t *test_registry_find(const char *id);
#endif
