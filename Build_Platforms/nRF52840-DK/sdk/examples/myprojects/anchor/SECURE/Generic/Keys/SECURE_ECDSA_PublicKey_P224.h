static const CRYPTO_MPI_LIMB _SECURE_ECDSA_PublicKey_P224_YX_aLimbs[] = {
  CRYPTO_MPI_LIMB_DATA4(0xA0, 0x4F, 0xCE, 0xB2),
  CRYPTO_MPI_LIMB_DATA4(0x97, 0xF1, 0xEC, 0xEA),
  CRYPTO_MPI_LIMB_DATA4(0xD0, 0xDA, 0x02, 0x62),
  CRYPTO_MPI_LIMB_DATA4(0x08, 0xFB, 0x01, 0x1C),
  CRYPTO_MPI_LIMB_DATA4(0xAC, 0x47, 0xF2, 0x2E),
  CRYPTO_MPI_LIMB_DATA4(0x39, 0x13, 0x43, 0x54),
  CRYPTO_MPI_LIMB_DATA4(0x0D, 0x92, 0xD4, 0x24)
};

static const CRYPTO_MPI_LIMB _SECURE_ECDSA_PublicKey_P224_YY_aLimbs[] = {
  CRYPTO_MPI_LIMB_DATA4(0xBF, 0x06, 0x10, 0x55),
  CRYPTO_MPI_LIMB_DATA4(0x03, 0xAC, 0x1C, 0x41),
  CRYPTO_MPI_LIMB_DATA4(0x32, 0xC2, 0x9D, 0xBA),
  CRYPTO_MPI_LIMB_DATA4(0xF3, 0xF0, 0x14, 0xEC),
  CRYPTO_MPI_LIMB_DATA4(0x9B, 0xA3, 0x92, 0x05),
  CRYPTO_MPI_LIMB_DATA4(0x6C, 0x60, 0x95, 0xCB),
  CRYPTO_MPI_LIMB_DATA4(0x18, 0xEF, 0x2D, 0x62)
};

static const CRYPTO_ECDSA_PUBLIC_KEY _SECURE_ECDSA_PublicKey_P224 = { {
  { CRYPTO_MPI_INIT_RO(_SECURE_ECDSA_PublicKey_P224_YX_aLimbs) },
  { CRYPTO_MPI_INIT_RO(_SECURE_ECDSA_PublicKey_P224_YY_aLimbs) },
  { CRYPTO_MPI_INIT_RO_ZERO },
  { CRYPTO_MPI_INIT_RO_ZERO },
  },
  &CRYPTO_EC_CURVE_secp224r1
};
