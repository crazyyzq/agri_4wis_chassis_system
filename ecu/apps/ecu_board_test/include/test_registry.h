/* Immutable ordered test catalogue and dependency metadata. */
#ifndef ECU_TEST_REGISTRY_H
#define ECU_TEST_REGISTRY_H
#include <stddef.h>
#include "test_runner.h"
/* Returned storage is static; count may be null and callers must not modify it. */
const test_descriptor_t *test_registry_all(size_t *count);
/* Return null for a null or unknown exact case-sensitive identifier. */
const test_descriptor_t *test_registry_find(const char *id);
#endif
