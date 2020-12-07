#include "image_processing.hpp"
using namespace std;

namespace improc {
	void pyrDownNormal(const cv::Mat& source, cv::Mat& dest) {
		const float* source_ptr = source.ptr<float>(0);
		float* dest_ptr = dest.ptr<float>(0);
		int width = source.cols;
		int height = source.rows;
		// normal
		const float* s;
		int wh = width*height;
		for (int v = 0; v < wh; v += 2 * width) {
			for (int u = 0; u < width; u += 2) {
				s = source_ptr + u + v;
				*dest_ptr = (s[0] + s[1] + s[width] + s[width + 1])*0.25f;
				++dest_ptr;
			}
		}
	};

	void pyrDownSSE(const cv::Mat& source, cv::Mat& dest) {
		const float* source_ptr = source.ptr<float>(0);
		float* dest_ptr = dest.ptr<float>(0);
		int width = source.cols;
		int height = source.rows;

		// SSE
		if (width % 8 == 0) {
			__m128 p025 = _mm_setr_ps(0.25f, 0.25f, 0.25f, 0.25f);
			const float* maxY = source_ptr + width*height; // žÇ ³¡ Æ÷ÀÎÅÍ
			for (const float* y = source_ptr; y < maxY; y += width * 2) { // µÎÁÙŸ¿ ³»·Á°š.
				const float* maxX = y + width; // ÇØŽç ÁÙ žÇ ³¡ Æ÷ÀÎÅÍ
				// _mm_prefetch((char*)maxX + 640, _MM_HINT_T0); // º° È¿°ú°¡ ŸøŽÙ.
				for (const float* x = y; x < maxX; x += 8) {
					__m128 top_left = _mm_load_ps((float*)x);
					__m128 bot_left = _mm_load_ps((float*)x + width);
					__m128 left = _mm_add_ps(top_left, bot_left);
					__m128 top_right = _mm_load_ps((float*)x + 4);
					__m128 bot_right = _mm_load_ps((float*)x + width + 4);
					__m128 right = _mm_add_ps(top_right, bot_right);

					__m128 sumA = _mm_shuffle_ps(left, right, _MM_SHUFFLE(2, 0, 2, 0));
					__m128 sumB = _mm_shuffle_ps(left, right, _MM_SHUFFLE(3, 1, 3, 1));

					__m128 sum = _mm_add_ps(sumA, sumB);
					sum = _mm_mul_ps(sum, p025);

					_mm_store_ps(dest_ptr, sum);
					dest_ptr += 4;
				}
			}
		}
		else pyrDownNormal(source, dest);
	};

	void imagePyramid(const cv::Mat& img, vector<cv::Mat>& img_pyr) {
		// img needs to be 'CV_32FC1'.
		if (img_pyr.size() > 0) {
			size_t max_lvl = img_pyr.size();
			img.copyTo(img_pyr[0]);
			for (size_t lvl = 1; lvl < max_lvl; lvl++) {
				pyrDownSSE(img_pyr[lvl - 1], img_pyr[lvl]);
			}
		}
		else throw std::runtime_error("In 'imagePyramid', img_pyr needs to be initialized!\n");
	};

	void interpImage(const cv::Mat& img, const vector<chk::Point2f>& pts, vector<float>& brightness, vector<int>& valid_vec) {
		brightness.resize(0);
		valid_vec.resize(0);

		const float* img_ptr = img.ptr<float>(0);
		size_t n_pts = pts.size();
		int n_cols = img.cols;
		int n_rows = img.rows;

		int is_valid = -1; // valid pixel ?

		float u_cur, v_cur; // float-precision coordinates
		int u_0, v_0; // truncated coordinates
		int v_0n_cols, v_0n_colsu_0;

		float I1, I2, I3, I4;
		float ax, ay, axay;

		// I1 ax / 1-ax I2 
		// ay   (v,u)
		//  
		// 1-ay
		// I3           I4

		for (size_t i = 0; i < n_pts; i++) {
			u_cur = pts[i].x;
			v_cur = pts[i].y;
			u_0 = (int)floor(u_cur);
			v_0 = (int)floor(v_cur);
			is_valid = 1;
			brightness.push_back(0);
			valid_vec.push_back(0);

			if (u_cur >= 0 && u_cur < n_cols - 1)
				ax = u_cur - (float)u_0;
			else if (u_cur == n_cols - 1) {
				u_0 = (n_cols - 1) - 1;
				ax = 0;
			}
			else if (u_cur > -1 && u_cur < 0) {
				u_0 = 1;
				ax = 1;
			}
			else is_valid = 0;
			if (v_cur >= 0 && v_cur < n_rows - 1)
				ay = v_cur - (float)v_0;
			else if (v_cur == n_rows - 1) {
				v_0 = (n_rows - 1) - 1;
				ay = 0;
			}
			else if (v_cur > -1 && v_cur < 0) {
				v_0 = 1;
				ay = 1;
			}
			else is_valid = 0;

			axay = ax*ay;
			if (is_valid) {
				v_0n_cols = v_0*n_cols;
				v_0n_colsu_0 = v_0n_cols + u_0;
				I1 = img_ptr[v_0n_colsu_0];
				I2 = img_ptr[v_0n_colsu_0 + 1];
				I3 = img_ptr[v_0n_colsu_0 + n_cols];
				I4 = img_ptr[v_0n_colsu_0 + n_cols + 1];

				brightness[i] += axay*(I1 - I2 - I3 + I4);
				brightness[i] += ax*(-I1 + I2);
				brightness[i] += ay*(-I1 + I3);
				brightness[i] += I1;
				valid_vec[i] = 1;
			}
			else {
				brightness[i] = -2;
				valid_vec[i] = 0;
			}
		}
	};

	void interpImage(const cv::Mat& img, const vector<chk::Point2f>& pts, chk::Point2f& pt_offset, float* brightness, int* valid_vec) {
		
		const float* img_ptr = img.ptr<float>(0);
		size_t n_pts = pts.size();
		int n_cols = img.cols;
		int n_rows = img.rows;

		int is_valid = -1; // valid pixel ?

		float u_cur, v_cur; // float-precision coordinates
		int u_0, v_0; // truncated coordinates
		int v_0n_cols, v_0n_colsu_0;

		float I1, I2, I3, I4;
		float ax, ay, axay;

		// I1 ax / 1-ax I2 
		// ay   (v,u)
		//  
		// 1-ay
		// I3           I4

		for (size_t i = 0; i < n_pts; i++) {
			u_cur = pts[i].x + pt_offset.x;
			v_cur = pts[i].y + pt_offset.y;
			u_0 = (int)floor(u_cur);
			v_0 = (int)floor(v_cur);
			is_valid = 1;
			brightness[i] = 0;
			valid_vec[i] = 0;

			if (u_cur >= 0 && u_cur < n_cols - 1)
				ax = u_cur - (float)u_0;
			else if (u_cur == n_cols - 1) {
				u_0 = (n_cols - 1) - 1;
				ax = 0;
			}
			else if (u_cur > -1 && u_cur < 0) {
				u_0 = 1;
				ax = 1;
			}
			else is_valid = 0;
			if (v_cur >= 0 && v_cur < n_rows - 1)
				ay = v_cur - (float)v_0;
			else if (v_cur == n_rows - 1) {
				v_0 = (n_rows - 1) - 1;
				ay = 0;
			}
			else if (v_cur > -1 && v_cur < 0) {
				v_0 = 1;
				ay = 1;
			}
			else is_valid = 0;

			axay = ax*ay;
			if (is_valid) {
				v_0n_cols = v_0*n_cols;
				v_0n_colsu_0 = v_0n_cols + u_0;
				I1 = img_ptr[v_0n_colsu_0];
				I2 = img_ptr[v_0n_colsu_0 + 1];
				I3 = img_ptr[v_0n_colsu_0 + n_cols];
				I4 = img_ptr[v_0n_colsu_0 + n_cols + 1];

				brightness[i] += axay*(I1 - I2 - I3 + I4);
				brightness[i] += ax*(-I1 + I2);
				brightness[i] += ay*(-I1 + I3);
				brightness[i] += I1;
				valid_vec[i] = 1;
			}
			else {
				brightness[i] = -2;
				valid_vec[i] = 0;
			}
		}
	};

    float interpImageSingle(const cv::Mat& img, const float& u, const float& v) {

        float* img_ptr = (float*)img.ptr<float>(0);
        int n_cols = img.cols;
        int n_rows = img.rows;

        // I1 ax / 1-ax I2 
        // ay   (v,u)
        //  
        // 1-ay
        // I3           I4
        float ax, ay;
        int u0 = (int)u; // truncated coordinates
        int v0 = (int)v;
        // cout << "u0 v0: " << u0 << ", " << v0 << ", ncols, nrows: " << n_cols << ", " << n_rows << endl;
        if ((u0 > 0) && (u0 < n_cols - 1)) ax = u - (float)u0;
        else return -1;
        if ((v0 >= 0) && (v0 < n_rows - 1)) ay = v - (float)v0;
        else return -1;

        float axay = ax*ay;
        int v0cols = v0*n_cols;
        int v0colsu0 = v0cols + u0;

        float I00, I01, I10, I11;
        I00 = img_ptr[v0colsu0];
        I01 = img_ptr[v0colsu0 + 1];
        I10 = img_ptr[v0colsu0 + n_cols];
        I11 = img_ptr[v0colsu0 + n_cols + 1];

        float res = ax*(I01 - I00) + ay*(I10 - I00) + axay*(-I01 + I00 + I11 - I10) + I00;
        
        return res;
    };

    void interpImageSingle3(const cv::Mat& img, const cv::Mat& du, const cv::Mat& dv, const float& u, const float& v, chk::Point3f& interp_) 
    {
        float* img_ptr = (float*)img.ptr<float>(0);
        float* du_ptr = (float*)du.ptr<float>(0);
        float* dv_ptr = (float*)dv.ptr<float>(0);

        int n_cols = img.cols;
        int n_rows = img.rows;

        // I1 ax / 1-ax I2 
        // ay   (v,u)
        //  
        // 1-ay
        // I3           I4
        float ax, ay;
        int u0 = (int)u; // truncated coordinates
        int v0 = (int)v;
        // cout << "u0 v0: " << u0 << ", " << v0 << ", ncols, nrows: " << n_cols << ", " << n_rows << endl;
        if ((u0 > 0) && (u0 < n_cols - 1))
            ax = u - (float)u0;
        else {
            interp_.x = -1;
            interp_.y = -1;
            interp_.z = -1;
            return;
        }
        if ((v0 >= 0) && (v0 < n_rows - 1))
            ay = v - (float)v0;
        else {
            interp_.x = -1;
            interp_.y = -1;
            interp_.z = -1;
            return;
        }
        float axay = ax*ay;
        int v0cols = v0*n_cols;
        int v0colsu0 = v0cols + u0;

        float I00, I01, I10, I11;
        float du00, du01, du10, du11;
        float dv00, dv01, dv10, dv11;

        I00 = img_ptr[v0colsu0];
        I01 = img_ptr[v0colsu0 + 1];
        I10 = img_ptr[v0colsu0 + n_cols];
        I11 = img_ptr[v0colsu0 + n_cols + 1];

        du00 = du_ptr[v0colsu0];
        du01 = du_ptr[v0colsu0 + 1];
        du10 = du_ptr[v0colsu0 + n_cols];
        du11 = du_ptr[v0colsu0 + n_cols + 1];

        dv00 = dv_ptr[v0colsu0];
        dv01 = dv_ptr[v0colsu0 + 1];
        dv10 = dv_ptr[v0colsu0 + n_cols];
        dv11 = dv_ptr[v0colsu0 + n_cols + 1];

        float res_img = ax*(I01 - I00) + ay*(I10 - I00) + axay*(-I01 + I00 + I11 - I10) + I00;
        float res_du  = ax*(du01 - du00) + ay*(du10 - du00) + axay*(-du01 + du00 + du11 - du10) + du00;
        float res_dv  = ax*(dv01 - dv00) + ay*(dv10 - dv00) + axay*(-dv01 + dv00 + dv11 - dv10) + dv00;

        interp_.x = res_img;
        interp_.y = res_du;
        interp_.z = res_dv;
    };

	void sampleImage(const cv::Mat& img, const vector<chk::Point2f>& pts, chk::Point2f& pt_offset, float* brightness,int* valid_vec) {
	
		const float* img_ptr = img.ptr<float>(0);
		size_t n_pts = pts.size();

		int is_valid = -1; // valid pixel ?

		int u_cur, v_cur; // 
		for (size_t i = 0; i < n_pts; i++) {
			u_cur = (int)(pts[i].x + pt_offset.x);
			v_cur = (int)(pts[i].y + pt_offset.y);
			
			is_valid = 1;
			if ((u_cur > 0) && (u_cur < img.cols) && ( v_cur > 0) && (v_cur < img.rows)) {
				brightness[i] = *(img_ptr+v_cur*img.cols +u_cur);
				valid_vec[i] = 1;
			}
			else {
				brightness[i] = -2;
				valid_vec[i] = 0;
			}
		}
	};

    void diffImage(const cv::Mat& img, cv::Mat& dimg, bool flag_dx, bool flag_dy) {// central difference.
        const float* img_ptr = img.ptr<float>(0);
        float* dimg_ptr = dimg.ptr<float>(0);

        // p + n_rows*n_cols - 1; (žÇ ³¡ ÇÈŒ¿)
        // p + v*n_cols;          (v Çà žÇ Ã¹ ÇÈŒ¿)
        // p + (v+1)*n_cols - 1;  (v Çà žÇ ³¡ ÇÈŒ¿)
        if ((flag_dx == true) & (flag_dy == false)) { // dx
            int n_cols = img.cols;
            int n_rows = img.rows;
            
            for (int v = 0; v < n_rows; v++) {
                float* p_dimg = dimg_ptr + v*n_cols;
                float* p_dimg_rowendm1 = dimg_ptr + (v + 1)*n_cols - 1;
                const float* p_img_next = img_ptr + v*n_cols + 2;
                const float* p_img_prev = img_ptr + v*n_cols;

                // first and last pixel in this row.
                *p_dimg_rowendm1 = 0;
                *p_dimg = 0;
                ++p_dimg;
                for (; p_dimg < p_dimg_rowendm1; p_dimg++, p_img_next++, p_img_prev++)
                {
                    *p_dimg = 0.5f*(*p_img_next - *p_img_prev);
                }
            }
        }
        else if ((flag_dx == false) & (flag_dy == true)) { // dy
            int n_cols = img.cols;
            int n_rows = img.rows;

            float* p_dimg     = dimg_ptr;
            float* p_dimg_end = dimg_ptr + n_cols*(n_rows - 1);
            const float* p_img_next = img_ptr + 2*n_cols;
            const float* p_img_prev = img_ptr;
            for (; p_dimg < dimg_ptr + n_cols; p_dimg++) {
                *p_dimg = 0;
            }
            for (; p_dimg < p_dimg_end;) 
            {
                float* p_dimg_rowmax = p_dimg + n_cols;
                for (; p_dimg < p_dimg_rowmax; p_dimg++, p_img_next++, p_img_prev++) 
                {
                    *p_dimg = 0.5f*(*p_img_next - *p_img_prev);
                }
            }
            for (; p_dimg < dimg_ptr + n_rows*n_cols; p_dimg++) {
                *p_dimg = 0;
            }
        }
        else { // error.
            throw std::runtime_error("Neither flag_dx nor flag_dy is true!\n");
        }
    };


    
	float calcZNCC(float* a, float* b, int len) {
		float mean_l = 0.0f;
		float mean_r = 0.0f;
		float numer = 0.0f;
		float denom_l = 0.0f;
		float denom_r = 0.0f;

		// calculate means
		for (int i = 0; i < len; i++) {
			mean_l += a[i];
			mean_r += b[i];
		}
		float invn_elem = 1.0f / ((float)len + 0.0001f);

		mean_l *= invn_elem;
		mean_r *= invn_elem;

		//calculate costs
		float l_minus_mean;
		float r_minus_mean;
		for (int i = 0; i < len; i++) {
			l_minus_mean = a[i] - mean_l;
			r_minus_mean = b[i] - mean_r;
			numer += l_minus_mean*r_minus_mean;
			denom_l += l_minus_mean*l_minus_mean;
			denom_r += r_minus_mean*r_minus_mean;
		}
		return numer / sqrt(denom_l*denom_r + 0.001);
	};
};
