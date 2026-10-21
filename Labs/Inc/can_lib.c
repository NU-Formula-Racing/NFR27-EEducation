#include "vcu_can.h"
#include <stdlib.h>
#include <string.h>

void add_cantx_message(tx_messages_t *tx_msgs, message_header_t msg) {
  tx_msgs->messages[tx_msgs->message_count].name = strdup(msg.name);
  tx_msgs->messages[tx_msgs->message_count].header = msg.header;
  tx_msgs->messages[tx_msgs->message_count].handler.construct_message =
      msg.handler.construct_message;
  tx_msgs->messages[tx_msgs->message_count].speed = msg.speed;

  tx_msgs->message_count++;
}

void add_canrx_message(rx_messages_t *rx_msgs, message_header_t msg) {
  rx_msgs->messages[rx_msgs->message_count].name = strdup(msg.name);
  rx_msgs->messages[rx_msgs->message_count].header = msg.header;
  rx_msgs->messages[rx_msgs->message_count].handler.parse_message =
      msg.handler.parse_message;

  rx_msgs->message_count++;
}

int configure_filter(rx_messages_t *rx_msgs,
                     CAN_HandleTypeDef *hcan) { // add bank offset later

  if (rx_msgs->message_count > 56) {
    printf("Exceed Max allocation, use bitmask instead\n");
  }

  CAN_FilterTypeDef canfilterconfig;

  for (uint8_t msg_idx = 0, filter_bank = 0; msg_idx < rx_msgs->message_count;
       msg_idx += 4, filter_bank++) {
    canfilterconfig.FilterBank = filter_bank;
    canfilterconfig.FilterMode = CAN_FILTERMODE_IDLIST;
    canfilterconfig.FilterScale = CAN_FILTERSCALE_16BIT;
    canfilterconfig.FilterActivation = ENABLE;
    canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    canfilterconfig.SlaveStartFilterBank = 14; // CAN2 starts at bank 14

    // Each 16-bit list filter bank holds up to 4 IDs.
    // STM32 16-bit ID list format: ID[10:0] shifted left by 5.
    uint16_t slot[4] = {0, 0, 0, 0};
    for (uint8_t s = 0; s < 4 && (msg_idx + s) < rx_msgs->message_count; s++) {
      slot[s] = (rx_msgs->messages[msg_idx + s].header.RxHeader_.StdId
                 << 5); // Standard ID: bits [15:5]
    }

    canfilterconfig.FilterIdHigh = slot[0];
    canfilterconfig.FilterIdLow = slot[1];
    canfilterconfig.FilterMaskIdHigh = slot[2];
    canfilterconfig.FilterMaskIdLow = slot[3];

    if (HAL_CAN_ConfigFilter(hcan, &canfilterconfig) != HAL_OK) {
      return HAL_ERROR;
    }
  }
  return HAL_OK;
}

void parse_canrx_message(rx_messages_t *rx_msgs, CAN_RxHeaderTypeDef *RxHeader,
                         uint8_t *rxData) {
  for (uint8_t i = 0; i < rx_msgs->message_count; i++) {
    if (rx_msgs->messages[i].handler.parse_message != NULL &&
        rx_msgs->messages[i].header.RxHeader_.StdId == RxHeader->StdId) {
      rx_msgs->messages[i].handler.parse_message(rxData);
      return;
    }
  }
}

void send_cantx_messages(CAN_HandleTypeDef *hcan, tx_messages_t *tx_msgs,
                         can_speed_t speed) {
  uint32_t TxMailBox = 0;
  for (uint8_t i = 0; i < tx_msgs->message_count; i++) {
    if (tx_msgs->messages[i].handler.construct_message != NULL &&
        tx_msgs->messages[i].speed == speed) {
      for (uint8_t c = 0; c < 8; c++) {
        tx_msgs->messages[i].TxData_[c] = 0;
      }
      tx_msgs->messages[i].handler.construct_message(&tx_msgs->messages[i]);
      if (send_can_message(hcan, &tx_msgs->messages[i].header.TxHeader_,
                           tx_msgs->messages[i].TxData_, &TxMailBox) != 0) {
        continue;
      }

      uint32_t start = HAL_GetTick();
      while (HAL_CAN_IsTxMessagePending(hcan, TxMailBox)) {
        if ((HAL_GetTick() - start) > 5U) {
          break;
        }
      }
    }
  }
}

uint8_t send_can_message(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *TxHeader,
                         uint8_t *data, uint32_t *TxMailBox) {
  if (HAL_CAN_AddTxMessage(hcan, TxHeader, data, TxMailBox) != HAL_OK) {
    return 1;
  }
  return 0;
}
