idf_component_register(INCLUDE_DIRS "c_library_v2")

# Biblioteka generuje ostrzeżenia o packed-member i nieużywanych static inline.
# Przy -Werror (typowe w CI) build padłby na cudzych nagłówkach — wyciszamy lokalnie.
# INTERFACE, bo to komponent header-only: flaga dotyczy włączających, nie targetu.
target_compile_options(${COMPONENT_LIB} INTERFACE -Wno-address-of-packed-member)
