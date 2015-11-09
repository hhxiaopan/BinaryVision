void ChangeToBinary(IplImage *src, IplImage *BinaryImage)
{
	//ÖµÍ¼Ïñ
	for (int i = 0; i <src->height; i++)
	{
		for (int j = 0; j <src->width; j++)
		{

			if (((uchar *)(src->imageData + i*src->widthStep))[j*src->nChannels + 2]>70 &&
				((uchar *)(src->imageData + i*src->widthStep))[j*src->nChannels + 1]<70 &&
				((uchar *)(src->imageData + i*src->widthStep))[j*src->nChannels + 3]<70
				)
				((uchar *)(BinaryImage->imageData + i*BinaryImage->widthStep))[j] = 255;
			else
				((uchar *)(BinaryImage->imageData + i*BinaryImage->widthStep))[j] = 0;
		}
	}
}