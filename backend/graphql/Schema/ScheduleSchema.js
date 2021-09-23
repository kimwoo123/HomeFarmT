const { gql } = require('apollo-server');

const ScheduleSchema = gql`
  type Query {
    allSchedule: [Schedule]
    findSchedule(schedule_title: String): Schedule
  }

  type Mutation {
    createSchedule(schedule_title: String, schedule_dec: String): Schedule
    updateSchedule(schedule_title: String, schedule_dec: String): Schedule
    deleteSchedule(schedule_title: String, schedule_dec: String): Schedule
  }

  type Schedule {
    scheduleid: Int!
    schedule_time: String!
    schedule_title: String
    schedule_des: String
  }

`;

module.exports = ScheduleSchema;
