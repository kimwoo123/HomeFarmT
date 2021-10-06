<template>
  <div>
    <input type="checkbox" :id="`switch-${ checkboxId }`" name="switch1" class="input__on-off">
    <label :for="`switch-${ checkboxId }`" class="label__on-off font-300" @click="switchBtnClick">
      <span class="marble"></span>
      <span class="on">on</span>
      <span class="off">off</span>
    </label>
  </div>
</template>

<script>
export default {
  name: 'SwitchBtn',
  props: {
    defaultValue: Number,
    checkboxId: Number,
  },
  mounted() {
    document.querySelector(`#switch-${ this.checkboxId }`).checked = this.defaultValue
  },
  methods: {
    switchBtnClick() {
      const isChecked = document.querySelector(`#switch-${ this.checkboxId }`).checked
      this.$emit('is-checked', !isChecked)
    }
  },
}
</script>

<style lang="scss" scoped>
  input[type="checkbox"] {
    display: none;
  }

  .label__on-off {
    overflow: hidden;
    position: relative;
    display: inline-block;
    width: 48px;
    height: 21px;
    -webkit-border-radius: 13px;
    -moz-border-radius: 13px;
    border-radius: 13px;
    background-color: $gray;
    color: #fff;
    font-weight: bold;
    cursor: pointer;
    -webkit-transition: all .3s;
    -moz-transition: all .3s;
    -ms-transition: all .3s;
    -o-transition: all .3s;
    transition: all .3s;
  }

  .label__on-off > * {
    vertical-align: 4px;
    -webkit-transition: all .3s;
    -moz-transition: all .3s;
    -ms-transition: all .3s;
    -o-transition: all .3s;
    transition: all .3s;
    font-size: 8px;
  }

  .label__on-off .marble {
    position: absolute;
    top: 3px;
    left: 3px;
    display: block;
    width: 15px;
    height: 15px;
    background-color: #fff;
    -webkit-border-radius: 50%;
    -moz-border-radius: 50%;
    border-radius: 50%;
  }

  .label__on-off .on {
    display: none;
    padding-left: 12px;
  }

  .label__on-off .off {
    padding-left: 25px;
    line-height: 21px;
  }

  .input__on-off:checked + .label__on-off {
    background-color: $primary;
  }

  .input__on-off:checked + .label__on-off .on {
    display: inline-block;
  }

  .input__on-off:checked + .label__on-off .off {
    display: none;
  }

  .input__on-off:checked + .label__on-off .marble {
    left: 30px;
  }
</style>