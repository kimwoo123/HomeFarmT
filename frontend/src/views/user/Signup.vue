<template>
  <div class="signup-container">
    <div class="nav-container">
      <font-awesome-icon icon="arrow-left" @click="$router.push({ name: 'Login' })" class="icon"/>
      <span>Signup</span>
      <font-awesome-icon icon="user" class="icon-display"/>
    </div>

    <div class="logo"></div>

    <div class="input-container">
      <font-awesome-icon icon="user" class="icon"/>
      <input type="email" v-model="email" placeholder="email">
      <font-awesome-icon icon="lock" class="icon"/>
      <input type="password" id="password" v-model="password" placeholder="password">
      <font-awesome-icon icon="lock" class="icon"/>
      <input type="passwordConfirmation" id="passwordConfirmation" v-model="passwordConfirmation" placeholder="passwordConfirmation">
    </div>

    <button @click="requestSignup()">Signup</button>
  </div>
</template>

<style lang="scss" scoped>
@import './Signup.scss';
</style>

<script>
import gql from 'graphql-tag'
// import { useMutation } from '@vue/apollo-composable'

export default {
  name: 'Signup',
  data() {
    return {
      email: '',
      password: '',
      passwordConfirmation: '',
      error: {
				email: false,
				password: false,
				passwordConfirmation: false,
			},
    }
  },
  methods:{
    requestSignup() {
      this.$apollo.mutate({
        mutation: gql`mutation ($email: String!, $password: String) {
          signUp(email: $email , password: $password) {
            email
            password
          }
        }`, 
        variables: {
          email: this.email,
          password: this.password
        }
        })
        .then((res) => {
          console.log(res, 'done')
        })
        .catch((err) => {
          console.log(err, 'no')
        })
      },
    },
  }
</script>