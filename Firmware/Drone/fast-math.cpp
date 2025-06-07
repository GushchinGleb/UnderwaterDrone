#include "fast-math.h"

#include <Arduino.h>

#include <avr/pgmspace.h>  // Required for PROGMEM

static const float sin_lookup_table_256[] = {
  0.00000000f, 0.00615998f, 0.01231971f, 0.01847897f, 0.02463754f, 0.03079517f, 0.03695163f, 0.04310668f, 0.04926011f, 0.05541166f, 0.06156111f, 0.06770822f, 0.07385277f, 0.07999451f, 0.08613322f, 0.09226866f,
  0.09840060f, 0.10452881f, 0.11065304f, 0.11677308f, 0.12288869f, 0.12899964f, 0.13510568f, 0.14120661f, 0.14730218f, 0.15339215f, 0.15947631f, 0.16555440f, 0.17162623f, 0.17769153f, 0.18375011f, 0.18980171f,
  0.19584610f, 0.20188306f, 0.20791236f, 0.21393377f, 0.21994707f, 0.22595201f, 0.23194839f, 0.23793596f, 0.24391450f, 0.24988379f, 0.25584361f, 0.26179370f, 0.26773387f, 0.27366388f, 0.27958348f, 0.28549251f,
  0.29139069f, 0.29727781f, 0.30315363f, 0.30901799f, 0.31487060f, 0.32071126f, 0.32653975f, 0.33235586f, 0.33815935f, 0.34395000f, 0.34972763f, 0.35549197f, 0.36124283f, 0.36697996f, 0.37270316f, 0.37841225f,
  0.38410696f, 0.38978711f, 0.39545247f, 0.40110281f, 0.40673792f, 0.41235763f, 0.41796169f, 0.42354986f, 0.42912197f, 0.43467781f, 0.44021714f, 0.44573978f, 0.45124549f, 0.45673409f, 0.46220535f, 0.46765909f,
  0.47309506f, 0.47851309f, 0.48391297f, 0.48929447f, 0.49465743f, 0.50000161f, 0.50532681f, 0.51063281f, 0.51591945f, 0.52118653f, 0.52643383f, 0.53166115f, 0.53686827f, 0.54205507f, 0.54722130f, 0.55236673f,
  0.55749118f, 0.56259453f, 0.56767654f, 0.57273698f, 0.57777566f, 0.58279246f, 0.58778709f, 0.59275949f, 0.59770936f, 0.60263658f, 0.60754085f, 0.61242217f, 0.61728019f, 0.62211478f, 0.62692577f, 0.63171303f,
  0.63647628f, 0.64121532f, 0.64593011f, 0.65062034f, 0.65528595f, 0.65992665f, 0.66454226f, 0.66913271f, 0.67369777f, 0.67823726f, 0.68275100f, 0.68723887f, 0.69170064f, 0.69613618f, 0.70054525f, 0.70492780f,
  0.70928353f, 0.71361244f, 0.71791422f, 0.72218871f, 0.72643590f, 0.73065543f, 0.73484731f, 0.73901129f, 0.74314719f, 0.74725491f, 0.75133425f, 0.75538510f, 0.75940734f, 0.76340067f, 0.76736510f, 0.77130044f,
  0.77520645f, 0.77908307f, 0.78293008f, 0.78674746f, 0.79053491f, 0.79429239f, 0.79801977f, 0.80171680f, 0.80538344f, 0.80901957f, 0.81262493f, 0.81619948f, 0.81974310f, 0.82325554f, 0.82673681f, 0.83018667f,
  0.83360505f, 0.83699179f, 0.84034675f, 0.84366983f, 0.84696090f, 0.85021985f, 0.85344648f, 0.85664082f, 0.85980260f, 0.86293173f, 0.86602813f, 0.86909169f, 0.87212229f, 0.87511975f, 0.87808400f, 0.88101500f,
  0.88391250f, 0.88677651f, 0.88960683f, 0.89240342f, 0.89516610f, 0.89789486f, 0.90058953f, 0.90325004f, 0.90587628f, 0.90846813f, 0.91102552f, 0.91354835f, 0.91603649f, 0.91848987f, 0.92090845f, 0.92329204f,
  0.92564058f, 0.92795402f, 0.93023223f, 0.93247521f, 0.93468273f, 0.93685478f, 0.93899131f, 0.94109225f, 0.94315743f, 0.94518679f, 0.94718033f, 0.94913793f, 0.95105952f, 0.95294499f, 0.95479435f, 0.95660746f,
  0.95838428f, 0.96012467f, 0.96182871f, 0.96349621f, 0.96512711f, 0.96672148f, 0.96827912f, 0.96980000f, 0.97128409f, 0.97273135f, 0.97414166f, 0.97551507f, 0.97685140f, 0.97815073f, 0.97941285f, 0.98063785f,
  0.98182565f, 0.98297620f, 0.98408943f, 0.98516536f, 0.98620385f, 0.98720497f, 0.98816860f, 0.98909473f, 0.98998332f, 0.99083441f, 0.99164784f, 0.99242365f, 0.99316180f, 0.99386227f, 0.99452502f, 0.99515009f,
  0.99573731f, 0.99628681f, 0.99679846f, 0.99727231f, 0.99770832f, 0.99810648f, 0.99846679f, 0.99878913f, 0.99907362f, 0.99932021f, 0.99952888f, 0.99969959f, 0.99983239f, 0.99992728f, 0.99998420f, 1.00000000f
};

static const float sqrt_lookup_table_193[] = {
  1.00000000f, 1.00778711f, 1.01550961f, 1.02317381f, 1.03078103f, 1.03833246f, 1.04582942f, 1.05327308f, 1.06066442f, 1.06800461f, 1.07529473f, 1.08253574f, 1.08972859f, 1.09687436f, 1.10397387f, 1.11102796f,
  1.11803758f, 1.12500358f, 1.13192666f, 1.13880765f, 1.14564729f, 1.15244639f, 1.15920556f, 1.16592562f, 1.17260706f, 1.17925072f, 1.18585718f, 1.19242692f, 1.19896078f, 1.20545924f, 1.21192276f, 1.21835208f,
  1.22474766f, 1.23110998f, 1.23743951f, 1.24373686f, 1.25000262f, 1.25623703f, 1.26244068f, 1.26861393f, 1.27475727f, 1.28087127f, 1.28695619f, 1.29301238f, 1.29904044f, 1.30504060f, 1.31101334f, 1.31695890f,
  1.32287788f, 1.32877040f, 1.33463693f, 1.34047771f, 1.34629333f, 1.35208380f, 1.35784960f, 1.36359096f, 1.36930835f, 1.37500191f, 1.38067210f, 1.38631892f, 1.39194298f, 1.39754438f, 1.40312338f, 1.40868032f,
  1.41421533f, 1.41972888f, 1.42522109f, 1.43069208f, 1.43614233f, 1.44157207f, 1.44698131f, 1.45237041f, 1.45773959f, 1.46308911f, 1.46841908f, 1.47372985f, 1.47902155f, 1.48429430f, 1.48954844f, 1.49478412f,
  1.50000155f, 1.50520086f, 1.51038218f, 1.51554596f, 1.52069211f, 1.52582085f, 1.53093255f, 1.53602707f, 1.54110491f, 1.54616594f, 1.55121052f, 1.55623877f, 1.56125081f, 1.56624687f, 1.57122695f, 1.57619131f,
  1.58114016f, 1.58607352f, 1.59099150f, 1.59589446f, 1.60078228f, 1.60565531f, 1.61051357f, 1.61535716f, 1.62018633f, 1.62500119f, 1.62980175f, 1.63458824f, 1.63936079f, 1.64411950f, 1.64886439f, 1.65359569f,
  1.65831351f, 1.66301799f, 1.66770911f, 1.67238712f, 1.67705202f, 1.68170404f, 1.68634319f, 1.69096971f, 1.69558358f, 1.70018482f, 1.70477378f, 1.70935035f, 1.71391463f, 1.71846688f, 1.72300708f, 1.72753537f,
  1.73205173f, 1.73655641f, 1.74104953f, 1.74553096f, 1.75000095f, 1.75445950f, 1.75890684f, 1.76334298f, 1.76776791f, 1.77218175f, 1.77658474f, 1.78097677f, 1.78535795f, 1.78972852f, 1.79408836f, 1.79843771f,
  1.80277646f, 1.80710495f, 1.81142294f, 1.81573069f, 1.82002831f, 1.82431579f, 1.82859313f, 1.83286059f, 1.83711815f, 1.84136581f, 1.84560370f, 1.84983182f, 1.85405040f, 1.85825944f, 1.86245883f, 1.86664879f,
  1.87082946f, 1.87500072f, 1.87916279f, 1.88331568f, 1.88745940f, 1.89159405f, 1.89571965f, 1.89983630f, 1.90394402f, 1.90804291f, 1.91213310f, 1.91621447f, 1.92028713f, 1.92435122f, 1.92840683f, 1.93245375f,
  1.93649232f, 1.94052255f, 1.94454432f, 1.94855785f, 1.95256305f, 1.95656013f, 1.96054912f, 1.96452987f, 1.96850264f, 1.97246742f, 1.97642422f, 1.98037314f, 1.98431408f, 1.98824739f, 1.99217284f, 1.99609053f,
  2.00000000f
 };

float f_cos_f(float value) {
  return f_sin_f(value - (F_PI / 2.f));
}

float f_sin_f(float x) {
  x *= F_1PI * 2.0f; // set range [0; 1]
  uint32_t* xp = (uint32_t*)&x;
  int8_t minus = *xp >> 31; // get sign bit
  *xp &= ~(1 << 31); // set number to positive
  if (x > 1.f) {
    x = 2.f - x;
  }

  const float id = x * 255.f;
  const uint8_t id_i = id; // id of the left value

  float l_val;
  float r_val;
  if (id == 255) {
    l_val = sin_lookup_table_256[255 - 1];
    r_val = sin_lookup_table_256[255 - 0];
  } else {
    l_val = sin_lookup_table_256[id_i]; // left lookup value
    r_val = sin_lookup_table_256[id_i + 1]; // right lookup value
  }

  const float diff = id - id_i; // real position of index

  float result = l_val + (r_val - l_val) * diff; // linear approximation
  uint32_t* result_p = (uint32_t*)&result;
  *result_p |= minus << 31; // set sign bit

  return result;
}

float f_speed(float t, float a, float d) {
  float h = t - a; // delta

  return f_sqrt(2 * d * h);
}

float f_sqrt(float v) {
  const uint32_t val = *(uint32_t*)&v; // set bin representation

  int8_t exp = (val >> 23 & 0xFF); // exponent

  // check edge cases (nan, inf)
  if ((uint8_t)exp == 0xFF) {
    return v;
  }

  exp -= 127; // remove shift
  uint8_t sign = val >> 31;
  uint32_t mantissa = val & 0x7FFFFFL;

  mantissa |= 0x800000L; // add 1. ... that ignores in IEEE. New range is [1, 2)

  mantissa <<= exp & 1; // if exponent is odd number increase mantissa 2 times because the lowest bit will be removed. New range is [2, 4)
  mantissa -= 0x800000L; // remove 1 from increased mantissa. New range is [1, 3)

  exp >>= 1;  // get "sqrt" from exp
  exp += 127; // shift exp back to IEEE format

  uint8_t addr = mantissa >> (25 - 8); // address of the cell in the lookup table (25 - 17 bits of mantissa)
  if (addr >= 192) { // some how overflow
    uint32_t result = sign << 31 | (uint8_t)exp << 23 | 0x7FFFFFL;

    return *(float*)&result;
  }

  const float value_low = sqrt_lookup_table_193[addr];
  const float value_high = sqrt_lookup_table_193[addr + 1];

  const float shift = (float)(mantissa & 0x1FFFFL) / (float)0x20000L; // get shift on linear approximation (last 17 bits of mantissa)
  const float value = value_low + (value_high - value_low) * shift; // get value on linear approximation

  uint32_t result = (uint32_t)sign << 31 | (uint32_t)exp << 23 | ((*(uint32_t*)&value) & 0x7FFFFFL); // generate new IEEE754 float
  return *(float*)&result;
}

float f_rsqrt(float v) {
	long i;
	float r; // result

	r  = v;
	i  = *(long*)&r;
	i  = 0x5f3759dfL - ( i >> 1 ); // bit representation is log2 of a IEEE number
	r  = *(float*) &i;

	r  = r * (1.5F - ( v * 0.5F * r * r ));

	return r;
}

struct f_vector2 f_point_on_segment(const struct f_vector2& position, const struct f_vector2& start, const struct f_vector2& end, const float& shift) {
  const struct f_vector2 seg_vec = { .x = end.x - start.x, .y = end.y - start.y };
  const struct f_vector2 pos_vec = { .x = position.x - start.x, .y = position.y - start.y };
  const float seg_pos_scalar = f_v_scalar(seg_vec, pos_vec);

  const float r_pos_vec_len = f_v_rlength(pos_vec);
  const float normal_pos_mult = seg_pos_scalar * r_pos_vec_len;
  const struct f_vector2 normal_pos = {.x = seg_vec.x * normal_pos_mult, .y = seg_vec.y * normal_pos_mult};

  const float r_seg_len = f_v_rlength(seg_vec);
  const struct f_vector2 seg_normalize = {.x = seg_vec.x * r_seg_len, .y = seg_vec.y * r_seg_len};
  const struct f_vector2 p_on_seg = {.x = normal_pos.x + seg_normalize.x * shift, .y = normal_pos.y + seg_normalize.y * shift};

  return p_on_seg;
}

float f_v_length(const struct f_vector2& a) {
  return f_sqrt(a.x * a.x + a.y * a.y);
}

float f_v_rlength(const struct f_vector2& a) {
  return f_rsqrt(a.x * a.x + a.y * a.y);
}

float f_v_scalar(const struct f_vector2& a, const struct f_vector2& b) {
  return a.x * b.x + a.y * b.y;
}