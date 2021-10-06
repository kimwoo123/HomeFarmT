import Vue from 'vue'
import VueRouter from 'vue-router'
import Signup from '../views/user/Signup.vue'
import Login from '../views/user/Login.vue'
import My from '../views/user/My.vue'
import Home from '../views/Home.vue'
import History from '../views/History.vue'

Vue.use(VueRouter)

const routes = [
  {
    path: '*',
    redirect: '/'
  },
  {
    path: '/',
    redirect: Home,
  },
  {
    path: '/signup',
    name: 'Signup',
    component: Signup
  },
  {
    path: '/login',
    name: 'Login',
    component: Login
  },
  {
    path: '/my',
    name: 'My',
    component: My
  },
  {
    path: '/home',
    name: 'Home',
    component: Home
  },
  // iot
  {
    path: '/iot',
    name: 'IoT',
    component: () => import('../views/IoT/IoT.vue')
  },
  {
    path: '/iot/new',
    name: 'IoTNew',
    component: () => import('../views/IoT/IoTNew.vue')
  },
  // schedule
  {
    path: '/schedule',
    name: 'Schedule',
    component: () => import('../views/Schedule/Schedule.vue')
  },
  {
    path: '/schedule/new',
    name: 'ScheduleNew',
    component: () => import('../views/Schedule/ScheduleNew.vue')
  },
  // patrol
  {
    path: '/patrol',
    name: 'Patrol',
    component: () => import('../views/Patrol/Patrol.vue')
  },
  {
    path: '/patrol/setting',
    name: 'PatrolSetting',
    component: () => import('../views/Patrol/PatrolSetting.vue')
  },
  // control 
  {
    path: '/control',
    name: 'Control',
    component: () => import('../views/Control/Control.vue')
  },
  // history
  {
    path: '/history',
    name: 'History',
    component: History
  },
]

const router = new VueRouter({
  mode: 'history',
  base: process.env.BASE_URL,
  routes
})

export default router
