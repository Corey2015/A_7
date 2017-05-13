void analyze_image(const uint8_t *img_buf,
                   double        *img_avg,
                   double        *img_dev,
                   double        *img_score)
{
    const size_t  scan_size = (DFS747_SENSOR_ROWS - 6) * (DFS747_SENSOR_COLS - 6);
    const uint8_t pix_th    = 0xF5;

    double pix_val;
    double pix_sum;
    double sqr_sum;
    int    pix_cnt;
    double avg;
    double dev;
    double score;
    int    r, c;

    pix_sum = 0.0;
    sqr_sum = 0.0;
    pix_cnt = 0;
    for (r = 3; r < DFS747_SENSOR_ROWS - 3; r++) {
        for (c = 3; c < DFS747_SENSOR_COLS - 3; c++) {
            pix_val = img_buf[r * DFS747_SENSOR_COLS + c];

            pix_sum += pix_val;
            sqr_sum += (pix_val * pix_val);

            if (pix_val < pix_th) {
                pix_cnt++;
            }
        }
    }

    avg   = pix_sum / scan_size;
    dev   = sqrt(sqr_sum / scan_size - (avg * avg));
    score = (pix_cnt * 100) / scan_size;

    ALOGD("Average = %0.3f, Deviation = %0.3f, Score = %0.3f\n",
          avg, dev, score);

    if (img_avg != NULL) {
        *img_avg = avg;
    }

    if (img_dev != NULL) {
        *img_avg = dev;
    }

    if (img_score != NULL) {
        *img_score = score;
    }
}

