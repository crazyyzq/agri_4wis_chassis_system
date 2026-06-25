/* Immutable ordered test catalogue and dependency metadata. */
#ifndef ECU_TEST_REGISTRY_H
#define ECU_TEST_REGISTRY_H
#include <stddef.h>
#include "test_runner.h"
/**
 * @brief Return the complete ordered test descriptor catalogue.
 *
 * @param count Optional destination receiving the number of descriptors.
 * @return Borrowed pointer to immutable, statically owned descriptor storage.
 *
 * @warning Callers must not modify or free the returned array.
 */
const test_descriptor_t *test_registry_all(size_t *count);
/**
 * @brief Find one registered test by exact case-sensitive identifier.
 *
 * @param id NUL-terminated test identifier.
 * @return Borrowed pointer to immutable static storage, or null for a null or
 *         unknown identifier.
 */
const test_descriptor_t *test_registry_find(const char *id);
#endif
