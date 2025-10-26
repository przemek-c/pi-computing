        df['duration_s'] = df.apply(
            lambda row: 
            (row['distance'] * TURNING_RADIUS / row['velocity'] if row['velocity'] != 0 else 0 
             if row['steering'] == rs.Steering.STRAIGHT 
             else row['distance'] * TURNING_RADIUS) / row['velocity'] if row['velocity'] != 0 else 0,
            axis=1