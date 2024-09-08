#pragma once

#include <cstdint>
#include <functional>
#include <initializer_list>
#include <stdexcept>
#include <vector>

#include "logger.hpp"
#include "rtklib.h"

namespace navp::filter {

struct Filter;
template <typename Item>
struct FilterItem;

enum class CompareOperator : uint8_t {
  Greater,
  Less,
  Equal,
  GreaterEqual,
  LessEqual,
  NotEqual,
};

enum class LogicOperator : uint8_t { Or, And };

struct Filter {
  virtual constexpr bool call_filter(obs_t* obs, bool logic_or = true) const noexcept {
    return true;
  }
};

template <typename Item>
struct FilterItem : Filter {
  FilterItem(CompareOperator cmp_op, const std::vector<Item>& items) noexcept {
    _construct_op(cmp_op);
    _construct_items(items);
  }
  FilterItem(CompareOperator cmp_op, std::vector<Item>&& items) noexcept {
    _construct_op(cmp_op);
    _construct_items(items);
  }
  FilterItem(CompareOperator cmp_op, const std::initializer_list<Item>& items) noexcept {
    _construct_op(cmp_op);
    _construct_items(items);
  }
  FilterItem(CompareOperator cmp_op, std::initializer_list<Item>&& items) noexcept {
    _construct_op(cmp_op);
    _construct_items(items);
  }

  constexpr bool filter_item(const Item& lhs_item, LogicOperator logic = LogicOperator::Or) const {
    if (_op_status_error()) {
      auto error_msg = "The Filter operator isn't initialized successfully";
      nav_error(error_msg);
      throw std::runtime_error(error_msg);
    }
    if (logic == LogicOperator::Or) {
      // when the filter logic is `or`
      // if one item in rhs_items passes the filter,return true
      for (const auto& rhs_item : this->rhs_items) {
        if (this->op(lhs_item, rhs_item)) {
          return true;
        }
      }
      return false;
    } else {
      // when the filter logic is `and`
      // all items in rhs_items pass the filter,return true
      for (const auto& rhs_item : this->rhs_items) {
        if (!this->op(lhs_item, rhs_item)) {
          return false;
        }
      }
      return true;
    }
  }

  std::function<bool(Item, Item)> op;
  std::vector<Item> rhs_items;

 private:
  void _construct_op(CompareOperator cmp_op) {
    switch (cmp_op) {
      case CompareOperator::Greater:
        this->op = std::greater<Item>();
        break;
      case CompareOperator::Less:
        this->op = std::less<Item>();
        break;
      case CompareOperator::Equal:
        this->op = std::equal_to<Item>();
        break;
      case CompareOperator::GreaterEqual:
        this->op = std::greater_equal<Item>();
        break;
      case CompareOperator::LessEqual:
        this->op = std::less_equal<Item>();
        break;
      case CompareOperator::NotEqual:
        this->op = std::not_equal_to<Item>();
        break;
    }
  }
  void _construct_items(const std::vector<Item>& rhs) { this->rhs_items = rhs; }
  void _construct_items(std::vector<Item>&& rhs) { this->rhs_items = std::move(rhs); }
  void _construct_items(const std::initializer_list<Item>& rhs) { this->rhs_items = rhs; }
  void _construct_items(std::initializer_list<Item>&& rhs) { this->rhs_items = std::move(rhs); }
  bool _op_status_error() const noexcept { return !this->op; }
};

}  // namespace navp::filter