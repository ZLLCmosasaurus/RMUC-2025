CAN_HandleTypeDef hcan;
uint32_t ID = 0x123;
uint32_t Mask_ID = 0x456;

can_filter_mask_config(&hcan, 0x02, ID, Mask_ID);