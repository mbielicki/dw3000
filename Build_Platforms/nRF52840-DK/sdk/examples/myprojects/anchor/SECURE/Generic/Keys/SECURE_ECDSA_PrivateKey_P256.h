static const CRYPTO_MPI_LIMB _SECURE_ECDSA_PrivateKey_P256_X_aLimbs[] = {
  CRYPTO_MPI_LIMB_DATA4(0x91, 0xC9, 0x0B, 0x85),
  CRYPTO_MPI_LIMB_DATA4(0x0F, 0x58, 0xD4, 0x46),
  CRYPTO_MPI_LIMB_DATA4(0x15, 0xA1, 0x3B, 0x4C),
  CRYPTO_MPI_LIMB_DATA4(0x17, 0x00, 0xEC, 0xE9),
  CRYPTO_MPI_LIMB_DATA4(0xC8, 0xB1, 0x62, 0x5A),
  CRYPTO_MPI_LIMB_DATA4(0x9F, 0x8D, 0x88, 0x8A),
  CRYPTO_MPI_LIMB_DATA4(0x52, 0x1F, 0x6C, 0xB5),
  CRYPTO_MPI_LIMB_DATA4(0x1E, 0x8D, 0x1B, 0xE0)
};

static const CRYPTO_ECDSA_PRIVATE_KEY _SECURE_ECDSA_PrivateKey_P256 = {
  { CRYPTO_MPI_INIT_RO(_SECURE_ECDSA_PrivateKey_P256_X_aLimbs) },
  &CRYPTO_EC_CURVE_secp256r1
};

